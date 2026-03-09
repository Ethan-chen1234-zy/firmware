#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(HAS_RAKHUB)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "RAKSensorHub.h"
#include "TelemetrySensor.h"
#include "concurrency/Periodic.h"
#include <RAK-OneWireSerial.h>
#include <onewire_master_protocol.h> // for DELIMTER/WAKEUPBYTE and frame layout

#include <mutex>
#include <queue>
#include <set>
#include <cstring>

using namespace concurrency;

#define BOOT_DATA_REQ

/** Construct RAK 1-Wire sensor hub (type SENSOR_UNSET, name "RAKSensorHub"); actual init in runOnce(). */
RAKSensorHub::RAKSensorHub() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "RAKSensorHub") {}

RAKSensorHub rakSensorHub;

static Periodic *onewireRxPeriodic;
static Periodic *onewirePollPeriodic;

static SoftwareHalfSerial mySerial(HALF_UART_PIN); // Wire pin  P0.15
static Lock onewireLock;

static uint8_t buff[0x200];
static uint16_t bufflen = 0;

// Power module cache (IPSO 0xB8 current / 0xB9 voltage / 0xBA capacity), read by getBusVoltageMv/getCurrentMa/getBusBatteryPercent
static HubPower power;

// Environment sensor cache; add field in EnvCache and assign in switch when adding new IPSO
static EnvCache env;

/** Write a scalar sensor reading: update value, valid flag and last update time (used when parsing IPSO into env.*). */
static inline void setScalar(ScalarReading &r, float v, uint32_t nowMs)
{
    r.value = v;
    r.valid = true;
    r.lastUpdateMs = nowMs;
}

/** Return true if scalar reading is within validity window: valid, non-zero timestamp, and not older than maxAgeMs. */
static inline bool scalarFresh(const ScalarReading &r, uint32_t nowMs, uint32_t maxAgeMs)
{
    return r.valid && r.lastUpdateMs != 0 && (nowMs - r.lastUpdateMs) <= maxAgeMs;
}

// ----- Probe and polling state -----
static std::set<uint8_t> provision_list;  // Set of registered probe PIDs (from ADD_PID / capability hot-plug)
static std::set<uint8_t> sid_seen;         // SIDs seen so far (reserved)
static uint32_t pid_delta = 0;             // Last time we advanced poll PID (for 1500 ms interval)
static uint32_t data_delta = 0;            // Reserved

// ----- Timestamps (RX/TX pacing, timeouts, idle detection) -----
static uint32_t last_poll_time = 0;   // Last time we sent get.data poll
static uint32_t last_tx_time = 0;     // Last byte sent (half-duplex: must not overlap RX)
static uint32_t last_rx_time = 0;     // Last byte received
static uint32_t last_byte_time = 0;   // Last byte written to buff (for idle discard)
static uint32_t last_err_time = 0;    // Last checksum/sequence error (briefly stop TX after error)
static uint32_t last_capability_time = 0; // Last time we saw a capability/provision-like frame (used to quiet TX during join)

// ----- Request/response state (half-duplex: only one outstanding request) -----
static bool awaiting_rsp = false;          // Waiting for probe response
static uint32_t awaiting_rsp_since = 0;   // Time request was sent; give up after 2 s and advance PID
static bool data_poll_pid_valid = false;  // Current poll PID is valid
static uint8_t data_poll_pid = 0;         // PID currently being polled
static bool last_sent_pid_valid = false;
static uint8_t last_sent_pid = 0;

// ----- Frame parse state (safe resync on checksum/seq error, avoid underflow) -----
static bool processing_frame = false;        // Inside process()
static bool frame_error_during_process = false; // CHKSUM/SEQ error occurred during process

static uint32_t sid_queue_delta = 0;  // Reserved
static uint32_t status_delta = 0;     // Last time we logged status (about every 5 s)

// Hot-plug: periodic listen window (no TX) so new probes can send capability without OVF/collision
static const uint32_t LISTEN_WINDOW_INTERVAL_MS = 60000;  // every 60 s
static const uint32_t LISTEN_WINDOW_DURATION_MS = 3000;   // 3 s no TX
static uint32_t last_listen_schedule = 0;
static uint32_t listen_window_until = 0;

/** Get first PID from provision list (poll start or reset when no current PID). */
static bool getFirstProvisionPid(uint8_t &pid)
{
    if (provision_list.empty()) {
        return false;
    }
    pid = *provision_list.begin();
    return true;
}

/** Get next PID after current in provision list (round-robin over probes). */
static bool getNextProvisionPid(uint8_t current, uint8_t &next)
{
    auto it = provision_list.upper_bound(current);
    if (it == provision_list.end()) {
        return false;
    }
    next = *it;
    return true;
}

/** Advance poll PID to next provisioned probe; wrap to first if at end (round-robin). */
static void advanceDataPollPid()
{
    if (provision_list.empty()) {
        data_poll_pid_valid = false;
        return;
    }

    if (!data_poll_pid_valid || provision_list.count(data_poll_pid) == 0) {
        data_poll_pid_valid = getFirstProvisionPid(data_poll_pid);
        return;
    }

    uint8_t nextPid = 0;
    if (getNextProvisionPid(data_poll_pid, nextPid)) {
        data_poll_pid = nextPid;
        data_poll_pid_valid = true;
    } else {
        data_poll_pid_valid = getFirstProvisionPid(data_poll_pid);
    }
}

/**
 * 1-Wire protocol event callback: invoked by RakSNHub_Protocl_API.process() after parsing a frame.
 * Handles REQ/RSP, ADD_PID/ADD_SID, QSEND (actual UART TX), SDATA_REQ/REPORT (IPSO parse -> env), checksum/seq errors.
 */
static void onewire_evt(const uint8_t pid, const uint8_t sid, const SNHUBAPI_EVT_E eid, uint8_t *msg, uint16_t len)
{
    switch (eid) {
    case SNHUBAPI_EVT_RECV_REQ:
        LOG_INFO("+EVT:PID[%02x],REQ", pid);
        break;
    case SNHUBAPI_EVT_RECV_RSP:
        LOG_INFO("+EVT:PID[%02x],RSP", pid);
        awaiting_rsp = false;
        advanceDataPollPid();
        break;

    case SNHUBAPI_EVT_QSEND:
        mySerial.write(msg, len);
        last_tx_time = millis();
        awaiting_rsp = true;
        awaiting_rsp_since = last_tx_time;
        LOG_DEBUG("RAKSensorHub: TX %u bytes (PID=0x%02x)", (unsigned)len, pid);
        break;

    case SNHUBAPI_EVT_ADD_SID:
        LOG_INFO("+ADD:SID:[%02x]", msg[0]);
        (void)sid;
        break;

    case SNHUBAPI_EVT_ADD_PID:
        LOG_INFO("+ADD:PID:[%02x]", msg[0]);
        provision_list.insert(msg[0]);
        data_poll_pid = msg[0];
        data_poll_pid_valid = true;
        break;

    case SNHUBAPI_EVT_GET_INTV:
        break;

    case SNHUBAPI_EVT_GET_ENABLE:
        LOG_INFO("+EVT:PID[%02x],ENABLE[%02x]", pid, msg[0]);
        break;

    case SNHUBAPI_EVT_SDATA_REQ:  // IPSO parse from sensor data request response
        LOG_INFO("+EVT:PID[%02x],IPSO[%02x]", pid, msg[0]);
        // for( uint16_t i=1; i<len; i++)
        // {
        //     LOG_INFO("%02x,", msg[i]);  
        // }
        // LOG_INFO("");
        switch (msg[0]) {
        case RAK_IPSO_TEMP_SENSOR: {  // Temperature (0x67), 0.1 °C
            if (len < 3)
                break;
            int16_t temp_raw = (msg[2] << 8) + msg[1];
            float temperature = temp_raw / 10.0f;
            if (temperature < -50.0f || temperature > 130.0f) {
                LOG_INFO("Ignore temperature sensor value out of range: %.2f C", temperature);
                break;
            }
            setScalar(env.temperature, temperature, millis());
            LOG_INFO("Temperature sensor: %.2f C", temperature);
            break;
        }
        case RAK_IPSO_HUMIDITY_SENSOR: {  // Humidity (0x68), integer %
            if (len < 2)
                break;
            uint8_t hum_raw = msg[1];
            // RAK2560 docs: IPSO 0x68 has 0.5 %RH resolution per bit (0..100 %RH -> raw 0..200)
            float humidity = hum_raw / 2.0f;
            if (humidity < 0.0f || humidity > 100.0f) {
                LOG_INFO("Ignore humidity sensor value out of range: %.2f %%", humidity);
                break;
            }
            setScalar(env.humidity, humidity, millis());
            LOG_INFO("Humidity sensor: %.2f %%", humidity);
            break;
        }
        case RAK_IPSO_HP_HUMIDITY: {  // High-precision humidity (0x70), 0.1 % RH
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            float moisture = raw / 10.0f;
            if (moisture < 0.0f || moisture > 100.0f) {
                LOG_INFO("Ignore high precision humidity value out of range: %.1f %%", moisture);
                break;
            }
            setScalar(env.soil_moisture, moisture, millis());
            LOG_INFO("High precision humidity: %.1f %%", moisture);
            break;
        }
        case RAK_IPSO_BAROMETER: {  // Barometric pressure (0x73), 0.1 hPa
            if (len < 3)
                break;
            int16_t press_raw = (msg[2] << 8) + msg[1];
            float pressure = press_raw / 10.0f;
            if (pressure < 100.0f || pressure > 1100.0f) {
                LOG_INFO("Ignore barometric pressure value out of range: %.1f hPa", pressure);
                break;
            }
            setScalar(env.pressure, pressure, millis());
            LOG_INFO("air pressure sensor: %.1f hPa", pressure);
            break;
        }
        case RAK_IPSO_CO2: {  // CO2 (0x7D), scale sensor-defined
            if (len < 3)
                break;
            uint16_t raw = (msg[2] << 8) + msg[1];
            float co2 = raw;
            if (co2 < 0.0f || co2 > 5000.0f) {
                LOG_INFO("Ignore CO2 sensor value out of range: %.2f ppm", co2);
                break;
            }
            setScalar(env.co2, co2, millis());
            LOG_INFO("CO2 sensor: %.2f ppm", co2);  
            break;
        }
        case RAK_IPSO_WIND: {  // Wind speed (0xBE), 0.01 m/s
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            float ws = raw / 100.0f;
            if (ws < 0.0f || ws > 60.0f) {
                LOG_INFO("Ignore wind speed value out of range: %.2f m/s", ws);
                break;
            }
            setScalar(env.wind_speed, ws, millis());
            LOG_INFO("Wind speed: %.2f m/s", ws);
            break;
        }
        case RAK_IPSO_WIND_DIR: {  // Wind direction (0xBF), 1°
            if (len < 3)
                break;
            uint16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.wind_direction, (float)(raw % 360), millis());
            LOG_INFO("Wind direction: %u deg", (unsigned)(raw % 360));
            break;
        }
        case RAK_IPSO_PYRANOMETER: {  // Pyranometer / radiation (0xC3)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.radiation, raw / 10.0f, millis());
            LOG_INFO("Pyranometer radiation: %.1f", env.radiation.value);
            break;
        }
        case RAK_IPSO_CAPACITY:  // Capacity percent (0–100 %)  0xBA
            if (len < 2)
                break;
            power.percent = msg[1];
            if (power.percent > 100) {
                power.percent = 100;
            }
            LOG_INFO("Battery capacity: %u %%", (unsigned)power.percent);
            break;
        case RAK_IPSO_DC_CURRENT:  // DC current mA 0xB8
            if (len < 3)
                break;
            power.curMa = (msg[2] << 8) + msg[1];
            LOG_INFO("Battery current raw: %d mA", (int)power.curMa);
            LOG_INFO("Battery current: %.3f A", (float)power.curMa / 1000.0f);
            break;
        case RAK_IPSO_DC_VOLTAGE:  // DC voltage mV (raw*10) 0xB9
            if (len < 3)
                break;
            power.volMv = (msg[2] << 8) + msg[1];
            power.volMv *= 10;
            LOG_INFO("Battery voltage raw: %u mV", (unsigned)power.volMv);
            LOG_INFO("Battery voltage: %.2f V", (float)power.volMv / 1000.0f);
            break;
        case RAK_IPSO_HP_PH: {  // High-precision pH (0xC1), 0.01 resolution
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            float ph = raw / 100.0f;
            setScalar(env.soil_ph, ph, millis());
            LOG_INFO("Soil pH: %.2f", ph);
            break;
        }
        case RAK_IPSO_ACCELEROMETER: {  // 3-axis accelerometer (0x71), 6 bytes X,Y,Z int16
            if (len < 7)
                break;
            int16_t x = (int16_t)((msg[2] << 8) + msg[1]);
            int16_t y = (int16_t)((msg[4] << 8) + msg[3]);
            int16_t z = (int16_t)((msg[6] << 8) + msg[5]);
            env.accel.x = x / 1000.0f;
            env.accel.y = y / 1000.0f;
            env.accel.z = z / 1000.0f;
            env.accel.valid = true;
            env.accel.lastUpdateMs = millis();
            LOG_INFO("Accelerometer (0x71): X=%.3f Y=%.3f Z=%.3f", env.accel.x, env.accel.y, env.accel.z);
            break;
        }
        case RAK_IPSO_SALINITY: {  // Salinity (0x13), 0.01 scale
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.salinity, raw / 100.0f, millis());
            LOG_INFO("Salinity: %.2f (raw units)", env.salinity.value);
            break;
        }
        case RAK_IPSO_EC: {  // Conductivity EC (0xC0), 0.001 mS/cm (0.001 µS/cm per bit)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            // According to High-Precision-EC spec: 0.001 µS/cm per bit -> 0.001 mS/cm
            setScalar(env.ec, raw / 1000.0f, millis());
            LOG_INFO("EC: %.3f (mS/cm units)", env.ec.value);
            break;
        }
        default:
            break;
        }
        rakSensorHub.setLastRead(millis());

        break;
    case SNHUBAPI_EVT_REPORT:  // Unsolicited report IPSO parse (same logic as SDATA_REQ)
        LOG_INFO("+EVT:PID[%02x],IPSO[%02x]", pid, msg[0]);
        // for( uint16_t i=1; i<len; i++)
        // {
        //     LOG_INFO("%02x,", msg[i]);
        // }
        // LOG_INFO("");

        switch (msg[0]) {
        case RAK_IPSO_TEMP_SENSOR: {  // Temperature (0x67)
            if (len < 3)
                break;
            int16_t temp_raw = (msg[2] << 8) + msg[1];
            float temperature = temp_raw / 10.0f;
            if (temperature < -50.0f || temperature > 130.0f) {
                LOG_INFO("Ignore temperature value out of range: %.2f C", temperature);
                break;
            }
            setScalar(env.temperature, temperature, millis());
            LOG_INFO("Temperature: %.2f C", temperature);
            break;
        }
        case RAK_IPSO_HUMIDITY_SENSOR: {  // Humidity (0x68)
            if (len < 2)
                break;
            uint8_t hum_raw = msg[1];
            // RAK2560 docs: IPSO 0x68 has 0.5 %RH resolution per bit (0..100 %RH -> raw 0..200)
            float humidity = hum_raw / 2.0f;
            if (humidity < 0.0f || humidity > 100.0f) {
                LOG_INFO("Ignore humidity value out of range: %.2f %%", humidity);
                break;
            }
            setScalar(env.humidity, humidity, millis());
            LOG_INFO("Humidity: %.2f %%", humidity);
            break;
        }
        case RAK_IPSO_HP_HUMIDITY: {  // High-precision humidity (0x70)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            float moisture = raw / 10.0f;
            if (moisture < 0.0f || moisture > 100.0f) {
                LOG_INFO("Ignore high precision humidity value out of range: %.1f %%", moisture);
                break;
            }
            setScalar(env.soil_moisture, moisture, millis());
            LOG_INFO("High precision humidity: %.1f %%", moisture);
            break;
        }
        case RAK_IPSO_BAROMETER: {  // Barometric pressure (0x73)
            if (len < 3)
                break;
            int16_t press_raw = (msg[2] << 8) + msg[1];
            float pressure = press_raw / 10.0f;
            if (pressure < 100.0f || pressure > 1100.0f) {
                LOG_INFO("Ignore barometric pressure value out of range: %.1f hPa", pressure);
                break;
            }
            setScalar(env.pressure, pressure, millis());
            LOG_INFO("air pressure sensor: %.1f hPa", pressure);
            break;
        }
        case RAK_IPSO_CO2: {  // CO2 (0x7D)
            if (len < 3)
                break;
            uint16_t raw = (msg[2] << 8) + msg[1];
            float co2 = raw;
            if (co2 < 0.0f || co2 > 5000.0f) {
                LOG_INFO("Ignore CO2 sensor value out of range: %.2f ppm", co2);
                break;
            }
            setScalar(env.co2, co2, millis());
            LOG_INFO("CO2 sensor: %.2f ppm", co2);
            break;
        }
        case RAK_IPSO_CAPACITY:  // Capacity percent
            if (len < 2)
                break;
            power.percent = msg[1];
            if (power.percent > 100) {
                power.percent = 100;
            }
            LOG_INFO("Battery capacity: %u %%", (unsigned)power.percent);
            break;
        case RAK_IPSO_DC_CURRENT:  // DC current
            if (len < 3)
                break;
            power.curMa = (msg[2] << 8) + msg[1];
            LOG_INFO("Battery current: %.3f A", (float)power.curMa / 1000.0f);
            break;
        case RAK_IPSO_DC_VOLTAGE:  // DC voltage
            if (len < 3)
                break;
            power.volMv = (msg[2] << 8) + msg[1];
            power.volMv *= 10;
            LOG_INFO("Battery voltage: %.2f V", (float)power.volMv / 1000.0f);
            break;
        case RAK_IPSO_WIND: {  // Wind speed (0xBE)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            float ws = raw / 100.0f;
            if (ws < 0.0f || ws > 60.0f) {
                LOG_INFO("Ignore wind speed value out of range: %.2f m/s", ws);
                break;
            }
            setScalar(env.wind_speed, ws, millis());
            LOG_INFO("Wind speed: %.2f m/s", ws);
            break;
        }
        case RAK_IPSO_WIND_DIR: {  // Wind direction (0xBF)
            if (len < 3)
                break;
            uint16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.wind_direction, (float)(raw % 360), millis());
            LOG_INFO("Wind direction: %u deg", (unsigned)(raw % 360));
            break;
        }
        case RAK_IPSO_PYRANOMETER: {  // Pyranometer (0xC3)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.radiation, raw / 10.0f, millis());
            LOG_INFO("Pyranometer radiation: %.1f", env.radiation.value);
            break;
        }
        case RAK_IPSO_HP_PH: {  // Soil pH (0xC1)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            float ph = raw / 100.0f;
            if (ph < 0.0f || ph > 14.0f) {
                LOG_INFO("Ignore soil pH value out of range: %.2f", ph);
                break;
            }
            setScalar(env.soil_ph, ph, millis());
            LOG_INFO("Soil pH: %.2f", ph);
            break;
        }
        case RAK_IPSO_ACCELEROMETER: {  // 3-axis accelerometer (0x71)
            if (len < 7)
                break;
            int16_t x = (int16_t)((msg[2] << 8) + msg[1]);
            int16_t y = (int16_t)((msg[4] << 8) + msg[3]);
            int16_t z = (int16_t)((msg[6] << 8) + msg[5]);
            env.accel.x = x / 1000.0f;
            env.accel.y = y / 1000.0f;
            env.accel.z = z / 1000.0f;
            env.accel.valid = true;
            env.accel.lastUpdateMs = millis();
            LOG_INFO("Accelerometer (0x71): X=%.3f Y=%.3f Z=%.3f", env.accel.x, env.accel.y, env.accel.z);
            break;
        }
        case RAK_IPSO_SALINITY: {  // Salinity (0x13), 0.01 scale
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.salinity, raw / 100.0f, millis());
            LOG_INFO("Salinity: %.2f (raw units)", env.salinity.value);
            break;
        }
        case RAK_IPSO_EC: {  // Conductivity EC (0xC0), 0.001 mS/cm (0.001 µS/cm per bit)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.ec, raw / 1000.0f, millis());
            LOG_INFO("EC: %.3f (mS/cm units)", env.ec.value);
            break;
        }
        default:
            break;
        }
        rakSensorHub.setLastRead(millis());

        break;

    case SNHUBAPI_EVT_CHKSUM_ERR:
        LOG_INFO("+ERR:CHKSUM");
        // The RX handler will drop the frame it just fed to process().
        // Touching bufflen here can cause uint16 underflow in the RX handler and
        // lead to infinite re-processing of the same corrupted frame.
        if (processing_frame) {
            frame_error_during_process = true;
        }
        last_err_time = millis();
        awaiting_rsp = false;
        break;

    case SNHUBAPI_EVT_SEQ_ERR:
        LOG_INFO("+ERR:SEQUCE");
        if (processing_frame) {
            frame_error_during_process = true;
        }
        last_err_time = millis();
        awaiting_rsp = false;
        break;

    default:
        break;
    }
}

/**
 * 1-Wire RX task (called periodically by Periodic): read bytes from half-duplex UART,
 * frame by 0x7E delimiter, call protocol process and onewire_evt. Includes overflow
 * recovery and capability-frame hot-plug PID parse. Returns suggested next call interval
 * (ms): 5 ms when active, 40 ms when idle.
 */
static int32_t onewireRxHandle()
{
    const uint32_t now = millis();
    std::lock_guard<Lock> guard(onewireLock);


    // Drain UART as fast as possible into our larger buffer
    while (mySerial.available()) {
        const uint8_t a = (uint8_t)mySerial.read();
        if (bufflen < sizeof(buff)) {
            buff[bufflen++] = a;
        } else {
            bufflen = 0; // overflow -> resync
        }
        last_byte_time = millis();
        last_rx_time = last_byte_time;
    }

    // If the underlying SoftwareHalfSerial ring overflowed, bytes were dropped.
    // Any partially assembled frame is now invalid -> hard resync.
    if (mySerial.overflow()) {
        LOG_INFO("+ERR:OVF");
        bufflen = 0;
        last_err_time = now;
        // After an overflow, briefly suppress TX (reuse listen window) so we only receive.
        listen_window_until = now + 300;
        return 5; // run again soon to drain and avoid repeated overflow
    }

    // Assemble complete frames using the declared payload length.
    while (bufflen > 0) {
        // Align to delimiter (0x7E). Protocol code will recover WAKEUPBYTE itself.
        uint16_t delim_idx = 0;
        while (delim_idx < bufflen && buff[delim_idx] != DELIMTER) {
            delim_idx++;
        }

        if (delim_idx >= bufflen) {
            // No delimiter yet; if we have been idle too long, drop noise.
            if ((now - last_byte_time) > 50) {
                bufflen = 0;
            }
            break;
        }

        if (delim_idx > 0) {
            memmove(buff, buff + delim_idx, bufflen - delim_idx);
            bufflen -= delim_idx;
            continue;
        }

        // Need at least: start + len(2) + type + flag
        if (bufflen < 5) {
            break;
        }

        // Length is stored as lbyte then hbyte, and the protocol interprets it as (lbyte<<8) + hbyte
        const uint16_t payload_len = ((uint16_t)buff[1] << 8) | buff[2];
        const uint16_t total_needed = 6 + payload_len; // 1(start)+2(len)+1(type)+1(flag)+payload_len+1(checksum)

        if (payload_len == 0 || total_needed > sizeof(buff)) {
            // Bad length -> discard one byte and resync
            memmove(buff, buff + 1, bufflen - 1);
            bufflen -= 1;
            continue;
        }

        if (bufflen < total_needed) {
            // Wait for the rest of the frame (do NOT process partial)
            break;
        }

        // Fallback: Parse capability frame (0x45/0x4D). Protocol lib may not emit ADD_PID (hot-plug).
        // PID at buff[34]; some probes use buff[33]=0xFF. 0xFF = broadcast -> assign next free PID.
        if (total_needed >= 36 && buff[3] == 0x02) {
            last_capability_time = now;
            uint8_t pid = buff[34];
            if (pid == 0 || pid == 0xFF)
                pid = buff[33];
            if (pid == 0xFF) {
                if (provision_list.count(0x01) == 0)
                    pid = 0x01;
                else {
                    pid = 0;
                    for (uint8_t q = 0x02; q < 0xFE; q++)
                        if (provision_list.count(q) == 0) {
                            pid = q;
                            break;
                        }
                }
            }
            if (pid != 0 && provision_list.count(pid) == 0) {
                provision_list.insert(pid);
                LOG_INFO("+BOOT:PID[%02x] from capability (len=%u) hot-plug", pid, (unsigned)payload_len);
            }
            // Quiet TX briefly after capability traffic to reduce chance of colliding with probe join/provision handshake.
            if (listen_window_until < now + 800) {
                listen_window_until = now + 800;
            }
        }

        // Lightweight frame prefix log (first 8 bytes) for field diagnosis.
        // Keep at DEBUG to avoid impacting 1-Wire timing (serial logging can cause RX overflow / join instability).
        if (total_needed >= 8) {
            LOG_DEBUG("+RX:len=%u %02x %02x %02x %02x %02x %02x %02x %02x", (unsigned)total_needed, (unsigned)buff[0],
                      (unsigned)buff[1], (unsigned)buff[2], (unsigned)buff[3], (unsigned)buff[4], (unsigned)buff[5],
                      (unsigned)buff[6], (unsigned)buff[7]);
        } else {
            LOG_DEBUG("+RX:len=%u", (unsigned)total_needed);
        }
        const uint16_t prev_len = bufflen;
        processing_frame = true;
        frame_error_during_process = false;
        RakSNHub_Protocl_API.process(buff, total_needed);
        processing_frame = false;

        if (frame_error_during_process) {
            // Robust resync: if checksum/sequence failed, we might have aligned to a
            // delimiter that is not the true frame start (or bytes were corrupted).
            // Drop one byte and search for the next delimiter.
            if (prev_len > 1) {
                memmove(buff, buff + 1, prev_len - 1);
                bufflen = (uint16_t)(prev_len - 1);
            } else {
                bufflen = 0;
            }
            last_err_time = now;
            continue;
        }

        // Success path: drop exactly this frame.
        const uint16_t remaining = (prev_len > total_needed) ? (uint16_t)(prev_len - total_needed) : 0;
        if (remaining > 0) {
            memmove(buff, buff + total_needed, remaining);
        }
        bufflen = remaining;
    }

    // At 9600 baud ~1 byte/ms. 0x45 (75B) and 0x4D (83B) frames need RX every ~5ms to avoid OVF,
    // but we can sleep longer when fully idle.
    const uint32_t idle_ms = 40;
    const uint32_t active_ms = 5;
    if ((now - last_byte_time) < 200)
        return (int32_t)active_ms;
    return (int32_t)idle_ms;
}

/**
 * 1-Wire poll task (called periodically): send get.data(pid) when link is idle.
 * Includes hot-plug listen window (3 s no TX every 60 s), discovery get.data(0x01..0x04),
 * and round-robin poll (~1.5 s per PID). Returns suggested next call interval (ms).
 */
static int32_t onewirePollHandle()
{
    const uint32_t now = millis();
    std::lock_guard<Lock> guard(onewireLock);

    // Additional information: If a buffer contains data and no new bytes are added for an extended period of time, the buffer is discarded (to prevent residual frames from blocking the buffer).
    if (bufflen > 0 && (now - last_byte_time) > 50) {
    bufflen = 0;
    }

    // Hot-plug: every 60s open a 3s listen window (no TX) so new probes can send capability frames
    if (now - last_listen_schedule >= LISTEN_WINDOW_INTERVAL_MS) {
        last_listen_schedule = now;
        listen_window_until = now + LISTEN_WINDOW_DURATION_MS;
        LOG_INFO("RAKSensorHub: listen window 3s (hot-plug)");
    }
    if (now < listen_window_until) {
        return 150; // no poll/TX; let onewireRxHandle receive unsolicited capability
    }
    // If we are seeing capability/provision traffic but haven't provisioned a PID yet, stay quiet for a short time.
    // This helps avoid repeated "register/provision" loops under heavy system load (e.g. when App is connected).
    if (provision_list.empty() && last_capability_time != 0 && (now - last_capability_time) < 1500) {
        return 150;
    }

    // Avoid transmitting while bytes are still arriving. At 9600bps one byte is ~1ms,
    // so a 10ms quiet gap is a reasonable "RX is done" heuristic.
    // After TX, wait 40ms so probe can send 0x0D response on half-duplex line
    const bool link_idle = bufflen == 0 && (now - last_rx_time) > 10 && (now - last_tx_time) > 40;

    // Only allow one outstanding request at a time; otherwise TX/RX overlap on half-duplex
    // can corrupt frames and trigger checksum errors.
    if (awaiting_rsp) {
        // If we don't get a response, fail open and keep polling.
        if ((now - awaiting_rsp_since) > 2000) {
            awaiting_rsp = false;
            last_err_time = now;
            advanceDataPollPid();
        } else {
            return 50;
        }
    }

    if (now - status_delta >= 5000) {
        status_delta = now;
        LOG_INFO("RAKSensorHub Status: last_tx: %lums ago, last_rx: %lums ago, provision=%u",
                 (unsigned long)(now - last_tx_time), (unsigned long)(now - last_rx_time),
                 (unsigned)provision_list.size());
    }

    // Discovery: when no PIDs, or periodically (hot-plug), try get.data(0x01..0x04) to trigger probe response
    static uint32_t last_discovery_time = 0;
    static uint8_t discovery_pid = 0x01;
    const bool run_discovery =
        (provision_list.empty() && (now - last_discovery_time) >= 5000) ||
        (!provision_list.empty() && (now - last_discovery_time) >= LISTEN_WINDOW_INTERVAL_MS);
    if (run_discovery && link_idle && (now - last_err_time) > 500) {
        last_discovery_time = now;
        RakSNHub_Protocl_API.get.data(discovery_pid);
        last_sent_pid = discovery_pid;
        last_sent_pid_valid = true;
        awaiting_rsp = true;
        awaiting_rsp_since = now;
        discovery_pid = (discovery_pid >= 0x04) ? 0x01 : (discovery_pid + 1);
        return 100;
    }

    // Regular sensor polling (temperature/humidity priority): one PID per tick.
    if (link_idle && provision_list.size() && (now - last_err_time) > 300 && now - pid_delta >= 1500) {
        pid_delta = now;
        if (!data_poll_pid_valid || provision_list.count(data_poll_pid) == 0) {
            data_poll_pid_valid = getFirstProvisionPid(data_poll_pid);
        }
        if (data_poll_pid_valid) {
            LOG_DEBUG("RAKSensorHub: poll get.data(PID=0x%02x)", data_poll_pid);
            RakSNHub_Protocl_API.get.data(data_poll_pid);
            last_sent_pid = data_poll_pid;
            last_sent_pid_valid = true;
            return 100;
        }
    }

    if (link_idle && provision_list.size() && (now - last_err_time) > 300 && now - last_poll_time >= 5000) {
        if (!data_poll_pid_valid || provision_list.count(data_poll_pid) == 0) {
            data_poll_pid_valid = getFirstProvisionPid(data_poll_pid);
        }
        if (data_poll_pid_valid) {
            RakSNHub_Protocl_API.get.data(data_poll_pid);
            last_sent_pid = data_poll_pid;
            last_sent_pid_valid = true;
        }
        last_poll_time = now;
        return 100;
    }

    return 150; // slower loop for command pacing
}

/** On first call: init 1-Wire UART, protocol, and RX/Poll Periodics; then just return default read interval. */
int32_t RAKSensorHub::runOnce()
{
    LOG_INFO("RAKSensorHub: runOnce...");
    if (!rakSensorHub.isInitialized()) {
         LOG_INFO("RAKSensorHub: Initializing OneWire sensor hub...");

        onewireRxPeriodic = new Periodic("onewireRxHandle", onewireRxHandle);
        onewirePollPeriodic = new Periodic("onewirePollHandle", onewirePollHandle);

        mySerial.begin(9600);

        RakSNHub_Protocl_API.init(onewire_evt);

        status = true;
        initialized = true;
    }

    return DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
}

/** Sensor setup placeholder; RAK 1-Wire hub needs no extra config. */
void RAKSensorHub::setup()
{
    // Set up oversampling and filter initialization
}

/** Fill measurement->variant.environment_metrics from env cache (temp, humidity, pressure, wind, soil, radiation, power). Returns true if any field was set. */
bool RAKSensorHub::getMetrics(meshtastic_Telemetry *measurement)
{
    bool any = false;
    const uint32_t now = millis();
    const uint32_t maxAgeMs = 60000;

    if (getBusVoltageMv() > 0) {
        measurement->variant.environment_metrics.has_voltage = true;   // Voltage in V from RAK power module (IPSO 0xB9 DC_VOLTAGE).
        measurement->variant.environment_metrics.has_current = true;   // Current in A from RAK power module (IPSO 0xB8 DC_CURRENT).
        measurement->variant.environment_metrics.voltage = (float)getBusVoltageMv() / 1000;   
        measurement->variant.environment_metrics.current = (float)getCurrentMa() / 1000;  
        any = true;
    }

    if (scalarFresh(env.temperature, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_temperature = true;
        measurement->variant.environment_metrics.temperature = env.temperature.value;   // Temperature in °C from RAK environmental sensor (IPSO 0x67 TEMPERATURE).
        any = true;
    }
    if (scalarFresh(env.humidity, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_relative_humidity = true;
        measurement->variant.environment_metrics.relative_humidity = env.humidity.value;   // Humidity in % from RAK environmental sensor (IPSO 0x68 RELATIVE_HUMIDITY).
        any = true;
    }
    if (scalarFresh(env.pressure, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_barometric_pressure = true;
        measurement->variant.environment_metrics.barometric_pressure = env.pressure.value;   // Pressure in hPa from RAK environmental sensor (IPSO 0x73 BAROMETRIC_PRESSURE).
        any = true;
    }
    if (scalarFresh(env.wind_speed, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_wind_speed = true;
        measurement->variant.environment_metrics.wind_speed = env.wind_speed.value;   // Wind speed in m/s from RAK environmental sensor (IPSO 0xBE WIND_SPEED).
        any = true;
    }
    if (scalarFresh(env.wind_direction, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_wind_direction = true;
        measurement->variant.environment_metrics.wind_direction = (uint16_t)(env.wind_direction.value <= 360 ? env.wind_direction.value : 0);   // Wind direction in degrees from RAK environmental sensor (IPSO 0xBF WIND_DIRECTION).
        any = true;
    }
    if (scalarFresh(env.soil_moisture, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_soil_moisture = true;
        float v = env.soil_moisture.value;
        measurement->variant.environment_metrics.soil_moisture = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : (uint32_t)v));   // Soil moisture in % from RAK environmental sensor (IPSO 0x70 HIGH_PRECISION_HUMIDITY).
        any = true;
    }
    if (scalarFresh(env.radiation, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_radiation = true;
        measurement->variant.environment_metrics.radiation = env.radiation.value;   // Radiation in W/m² from RAK environmental sensor (IPSO 0xC3 PYRANOMETER).
        any = true;
    }

    return any;
}

/** Bus voltage in mV from RAK power module (IPSO 0xB9 DC_VOLTAGE). */
uint16_t RAKSensorHub::getBusVoltageMv()
{
    return power.volMv;
}

/** Bus current in mA from RAK power module (IPSO 0xB8 DC_CURRENT). */
int16_t RAKSensorHub::getCurrentMa()
{
    return power.curMa;
}

/** Battery capacity 0..100 % from RAK power module (IPSO 0xBA CAPACITY). */
int RAKSensorHub::getBusBatteryPercent()
{
    return (int)power.percent;
}

/** True if current > 0 (charging). */
bool RAKSensorHub::isCharging()
{
    return (power.curMa > 0) ? true : false;
}

/** Called from onewire_evt when valid sensor data is received; updates TelemetrySensor lastRead. */
void RAKSensorHub::setLastRead(uint32_t lastRead)
{
    this->lastRead = lastRead;
}

#endif // HAS_RAKHUB  
