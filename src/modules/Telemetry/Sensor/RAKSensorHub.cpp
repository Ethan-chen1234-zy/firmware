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

/** 构造 RAK 一线传感器 Hub，类型为 SENSOR_UNSET，名称 "RAKSensorHub"；实际初始化在 runOnce() 中完成 */
RAKSensorHub::RAKSensorHub() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "RAKSensorHub") {}

RAKSensorHub rakSensorHub;

static Periodic *onewireRxPeriodic;
static Periodic *onewirePollPeriodic;

static SoftwareHalfSerial mySerial(HALF_UART_PIN); // Wire pin  P0.15
static Lock onewireLock;

static uint8_t buff[0x200];
static uint16_t bufflen = 0;

// 电源模块缓存（IPSO 0xB8 电流 / 0xB9 电压 / 0xBA 电量），由 getBusVoltageMv/getCurrentMa/getBusBatteryPercent 读出
static HubPower power;

// 环境传感器缓存：统一结构体，新增 IPSO 只需在 EnvCache 中加字段并在 switch 里赋值
static EnvCache env;

/** 写入单值传感器读数：更新数值、有效标志与最后更新时间（用于 IPSO 解析时写 env.*） */
static inline void setScalar(ScalarReading &r, float v, uint32_t nowMs)
{
    r.value = v;
    r.valid = true;
    r.lastUpdateMs = nowMs;
}

/** 判断单值读数是否在有效期内：有效且非零时间戳且未超过 maxAgeMs */
static inline bool scalarFresh(const ScalarReading &r, uint32_t nowMs, uint32_t maxAgeMs)
{
    return r.valid && r.lastUpdateMs != 0 && (nowMs - r.lastUpdateMs) <= maxAgeMs;
}

// ----- 探头与轮询状态 -----
static std::set<uint8_t> provision_list;  // 已注册的探头 PID 集合（来自 ADD_PID / 能力帧热插拔）
static std::set<uint8_t> sid_seen;        // 已见过的 SID，预留扩展
static int pid_delta = 0;                 // 上次推进轮询 PID 的时间戳，用于 1500ms 间隔
static int data_delta = 0;                // 预留

// ----- 时间戳（RX/TX 节奏、超时、空闲判断） -----
static uint32_t last_poll_time = 0;   // 上次发起 get.data 轮询的时间
static uint32_t last_tx_time = 0;     // 上次发送字节时间（半双工需与 RX 错开）
static uint32_t last_rx_time = 0;     // 上次收到任意字节时间
static uint32_t last_byte_time = 0;   // 上次往 buff 写入字节时间（用于空闲丢弃残留）
static uint32_t last_err_time = 0;    // 上次校验/序号错误时间，用于错误后短暂不再发请求

// ----- 请求/响应状态（半双工一次只允许一个未完成请求） -----
static bool awaiting_rsp = false;         // 是否正在等待探头响应
static uint32_t awaiting_rsp_since = 0;  // 发出请求的时间，超时 2s 后放弃并推进 PID
static bool data_poll_pid_valid = false; // 当前轮询用 PID 是否有效
static uint8_t data_poll_pid = 0;        // 当前轮询的 PID
static bool last_sent_pid_valid = false;
static uint8_t last_sent_pid = 0;

// ----- 帧解析状态（校验/序号错误时安全 resync，避免 underflow） -----
static bool processing_frame = false;       // 是否正在 process() 内
static bool frame_error_during_process = false; // process 过程中是否发生 CHKSUM/SEQ 错误

static int sid_queue_delta = 0;  // 预留
static int status_delta = 0;     // 上次打印状态日志的时间戳（约 5s 一次）

// Hot-plug: periodic listen window (no TX) so new probes can send capability without OVF/collision
static const uint32_t LISTEN_WINDOW_INTERVAL_MS = 60000;  // every 60s
static const uint32_t LISTEN_WINDOW_DURATION_MS = 3000;  // 3s no TX
static uint32_t last_listen_schedule = 0;
static uint32_t listen_window_until = 0;

/** 从已注册探头列表中取第一个 PID，用于轮询起点或无当前 PID 时复位 */
static bool getFirstProvisionPid(uint8_t &pid)
{
    if (provision_list.empty()) {
        return false;
    }
    pid = *provision_list.begin();
    return true;
}

/** 取当前 PID 在已注册列表中的下一个 PID，用于轮询时依次遍历所有探头 */
static bool getNextProvisionPid(uint8_t current, uint8_t &next)
{
    auto it = provision_list.upper_bound(current);
    if (it == provision_list.end()) {
        return false;
    }
    next = *it;
    return true;
}

/** 将轮询用 PID 推进到下一个已注册探头；若已是最后一个则回到第一个（实现 Round-Robin 轮询） */
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
 * 一线协议事件回调：由 RakSNHub_Protocl_API.process() 在解析完一帧后调用。
 * 处理 REQ/RSP、ADD_PID/ADD_SID、QSEND（实际发串口）、SDATA_REQ/REPORT（IPSO 解析写 env）、校验/序号错误等。
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

    case SNHUBAPI_EVT_SDATA_REQ:  // 传感器数据请求响应中的 IPSO 解析
        LOG_INFO("+EVT:PID[%02x],IPSO[%02x]", pid, msg[0]);
        // for( uint16_t i=1; i<len; i++)
        // {
        //     LOG_INFO("%02x,", msg[i]);  
        // }
        // LOG_INFO("");
        switch (msg[0]) {
        case RAK_IPSO_TEMP_SENSOR: {  // 温度传感器 (0x67)，0.1°C
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
        case RAK_IPSO_HUMIDITY_SENSOR: {  // 相对湿度 (0x68)，整型 %
            if (len < 2)
                break;
            uint8_t hum_raw = msg[1];
            float humidity = hum_raw / 1.0f;
            if (humidity < 0.0f || humidity > 100.0f) {
                LOG_INFO("Ignore humidity sensor value out of range: %.2f %%", humidity);
                break;
            }
            setScalar(env.humidity, humidity, millis());
            LOG_INFO("Humidity sensor: %.2f %%", humidity);
            break;
        }
        case RAK_IPSO_HP_HUMIDITY: {  // 高精度湿度 (0x70)，0.1%RH，气象站/土壤含水
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
        case RAK_IPSO_BAROMETER: {  // 气压 (0x73)，0.1 hPa
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
        case RAK_IPSO_CO2: {  // CO2 浓度 (0x7D)，单位/缩放由传感器定义
            if (len < 3)
                break;
            uint16_t raw = (msg[2] << 8) + msg[1];
            LOG_INFO("CO2 sensor: %u (raw units)", (unsigned)raw);
            break;
        }
        case RAK_IPSO_WIND: {  // 风速 (0xBE)，0.01 m/s
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
        case RAK_IPSO_WIND_DIR: {  // 风向 (0xBF)，1°
            if (len < 3)
                break;
            uint16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.wind_direction, (float)(raw % 360), millis());
            LOG_INFO("Wind direction: %u deg", (unsigned)(raw % 360));
            break;
        }
        case RAK_IPSO_PYRANOMETER: {  // 日照强度/辐射 (0xC3)，单位依传感器
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.radiation, raw / 10.0f, millis());
            LOG_INFO("Pyranometer radiation: %.1f", env.radiation.value);
            break;
        }
        case RAK_IPSO_CAPACITY:  // 电量百分比 (0~100%)
            if (len < 2)
                break;
            power.percent = msg[1];
            if (power.percent > 100) {
                power.percent = 100;
            }
            LOG_INFO("Battery capacity: %u %%", (unsigned)power.percent);
            break;
        case RAK_IPSO_DC_CURRENT:  // 直流电流，单位 mA
            if (len < 3)
                break;
            power.curMa = (msg[2] << 8) + msg[1];
            LOG_INFO("Battery current raw: %d mA", (int)power.curMa);
            LOG_INFO("Battery current: %.3f A", (float)power.curMa / 1000.0f);
            break;
        case RAK_IPSO_DC_VOLTAGE:  // 直流电压，单位 mV（raw*10）
            if (len < 3)
                break;
            power.volMv = (msg[2] << 8) + msg[1];
            power.volMv *= 10;
            LOG_INFO("Battery voltage raw: %u mV", (unsigned)power.volMv);
            LOG_INFO("Battery voltage: %.2f V", (float)power.volMv / 1000.0f);
            break;
        case RAK_IPSO_HP_PH: {  // 高精度 pH (0xC1)，0.01 分辨率
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            float ph = raw / 100.0f;
            setScalar(env.soil_ph, ph, millis());
            LOG_INFO("Soil pH: %.2f", ph);
            break;
        }
        case RAK_IPSO_ACCELEROMETER: {  // 三轴加速度 (0x71)，6 字节 X,Y,Z int16
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
        case RAK_IPSO_SALINITY: {  // 盐度 (0x13)，0.01 缩放（依据传感器）
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.salinity, raw / 100.0f, millis());
            LOG_INFO("Salinity: %.2f (raw units)", env.salinity.value);
            break;
        }
        case RAK_IPSO_EC: {  // 电导率 EC (0xC0)，0.01 缩放（依据传感器）
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.ec, raw / 100.0f, millis());
            LOG_INFO("EC: %.2f (raw units)", env.ec.value);
            break;
        }
        default:
            break;
        }
        rakSensorHub.setLastRead(millis());

        break;
    case SNHUBAPI_EVT_REPORT:  // 主动上报的 IPSO 解析（与 SDATA_REQ 逻辑一致）
        LOG_INFO("+EVT:PID[%02x],IPSO[%02x]", pid, msg[0]);
        // for( uint16_t i=1; i<len; i++)
        // {
        //     LOG_INFO("%02x,", msg[i]);
        // }
        // LOG_INFO("");

        switch (msg[0]) {
        case RAK_IPSO_TEMP_SENSOR: {  // 温度传感器 (0x67)
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
        case RAK_IPSO_HUMIDITY_SENSOR: {  // 相对湿度 (0x68)
            if (len < 2)
                break;
            uint8_t hum_raw = msg[1];
            float humidity = hum_raw / 1.0f;
            if (humidity < 0.0f || humidity > 100.0f) {
                LOG_INFO("Ignore humidity value out of range: %.2f %%", humidity);
                break;
            }
            setScalar(env.humidity, humidity, millis());
            LOG_INFO("Humidity: %.2f %%", humidity);
            break;
        }
        case RAK_IPSO_HP_HUMIDITY: {  // 高精度湿度 (0x70)
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
        case RAK_IPSO_BAROMETER: {  // 气压 (0x73)
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
        case RAK_IPSO_CO2: {  // CO2 浓度 (0x7D)
            if (len < 3)
                break;
            uint16_t raw = (msg[2] << 8) + msg[1];
            LOG_INFO("CO2 sensor: %u (raw units)", (unsigned)raw);
            break;
        }
        case RAK_IPSO_CAPACITY:  // 电量百分比
            if (len < 2)
                break;
            power.percent = msg[1];
            if (power.percent > 100) {
                power.percent = 100;
            }
            LOG_INFO("Battery capacity: %u %%", (unsigned)power.percent);
            break;
        case RAK_IPSO_DC_CURRENT:  // 直流电流
            if (len < 3)
                break;
            power.curMa = (msg[1] << 8) + msg[2];
            LOG_INFO("Battery current: %.3f A", (float)power.curMa / 1000.0f);
            break;
        case RAK_IPSO_DC_VOLTAGE:  // 直流电压
            if (len < 3)
                break;
            power.volMv = (msg[1] << 8) + msg[2];
            power.volMv *= 10;
            LOG_INFO("Battery voltage: %.2f V", (float)power.volMv / 1000.0f);
            break;
        case RAK_IPSO_WIND: {  // 风速 (0xBE)
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
        case RAK_IPSO_WIND_DIR: {  // 风向 (0xBF)
            if (len < 3)
                break;
            uint16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.wind_direction, (float)(raw % 360), millis());
            LOG_INFO("Wind direction: %u deg", (unsigned)(raw % 360));
            break;
        }
        case RAK_IPSO_PYRANOMETER: {  // 日照强度 (0xC3)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.radiation, raw / 10.0f, millis());
            LOG_INFO("Pyranometer radiation: %.1f", env.radiation.value);
            break;
        }
        case RAK_IPSO_HP_PH: {  // 土壤 pH (0xC1)
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
        case RAK_IPSO_ACCELEROMETER: {  // 三轴加速度 (0x71)
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
        case RAK_IPSO_SALINITY: {  // 盐度 (0x13)，0.01 缩放（依据传感器）
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.salinity, raw / 100.0f, millis());
            LOG_INFO("Salinity: %.2f (raw units)", env.salinity.value);
            break;
        }
        case RAK_IPSO_EC: {  // 电导率 EC (0xC0)，0.01 缩放（依据传感器）
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            setScalar(env.ec, raw / 100.0f, millis());
            LOG_INFO("EC: %.2f (raw units)", env.ec.value);
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
 * 一线 RX 任务（由 Periodic 周期性调用）：从半双工串口读入字节、按 0x7E 定界组帧、
 * 调用 protocol process 解析并触发 onewire_evt。含溢出恢复、能力帧热插拔 PID 解析。
 * 返回值：建议下次调用间隔(ms)，有数据时 5ms、空闲时 40ms，以平衡及时性与 CPU 占用。
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
        }

        // Lightweight frame prefix log (first 8 bytes) for field diagnosis
        if (total_needed >= 8) {
            LOG_INFO("+RX:len=%u %02x %02x %02x %02x %02x %02x %02x %02x", (unsigned)total_needed, (unsigned)buff[0],
                     (unsigned)buff[1], (unsigned)buff[2], (unsigned)buff[3], (unsigned)buff[4], (unsigned)buff[5],
                     (unsigned)buff[6], (unsigned)buff[7]);
        } else {
            LOG_INFO("+RX:len=%u", (unsigned)total_needed);
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
 * 一线轮询任务（由 Periodic 周期性调用）：在链路空闲时按周期发 get.data(pid) 拉取传感器数据。
 * 含热插拔监听窗口（60s 一次 3s 不发送）、发现阶段 get.data(0x01..0x04)、常规 Round-Robin 轮询（约 1.5s/pid）。
 * 返回值：建议下次调用间隔(ms)，用于控制发送节奏与半双工避让。
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

/** 首次调用时初始化一线串口、协议与 RX/Poll 两个 Periodic 任务；之后仅返回默认读间隔 */
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

/** 传感器初始化预留接口，当前 RAK 一线 Hub 无需额外配置 */
void RAKSensorHub::setup()
{
    // Set up oversampling and filter initialization
}

/** 将 env 缓存中未过期的环境量填入 measurement->variant.environment_metrics（温湿度、气压、风速风向、土壤湿度、辐射等）；含电源电压/电流。返回是否有任意字段被填充 */
bool RAKSensorHub::getMetrics(meshtastic_Telemetry *measurement)
{
    bool any = false;
    const uint32_t now = millis();
    const uint32_t maxAgeMs = 60000;

    if (getBusVoltageMv() > 0) {
        measurement->variant.environment_metrics.has_voltage = true;
        measurement->variant.environment_metrics.has_current = true;
        measurement->variant.environment_metrics.voltage = (float)getBusVoltageMv() / 1000;
        measurement->variant.environment_metrics.current = (float)getCurrentMa() / 1000;
        any = true;
    }

    if (scalarFresh(env.temperature, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_temperature = true;
        measurement->variant.environment_metrics.temperature = env.temperature.value;
        any = true;
    }
    if (scalarFresh(env.humidity, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_relative_humidity = true;
        measurement->variant.environment_metrics.relative_humidity = env.humidity.value;
        any = true;
    }
    if (scalarFresh(env.pressure, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_barometric_pressure = true;
        measurement->variant.environment_metrics.barometric_pressure = env.pressure.value;
        any = true;
    }
    if (scalarFresh(env.wind_speed, now, maxAgeMs)) {
        measurement->variant.environment_metrics.wind_speed = env.wind_speed.value;
        any = true;
    }
    if (scalarFresh(env.wind_direction, now, maxAgeMs)) {
        measurement->variant.environment_metrics.wind_direction = (uint32_t)env.wind_direction.value;
        any = true;
    }
    if (scalarFresh(env.soil_moisture, now, maxAgeMs)) {
        measurement->variant.environment_metrics.soil_moisture = (uint32_t)env.soil_moisture.value;
        any = true;
    }
    if (scalarFresh(env.radiation, now, maxAgeMs)) {
        measurement->variant.environment_metrics.radiation = env.radiation.value;
        any = true;
    }

    return any;
}

/** 返回 RAK 电源模块上报的母线电压，单位 mV（来自 IPSO 0xB9 DC_VOLTAGE） */
uint16_t RAKSensorHub::getBusVoltageMv()
{
    return power.volMv;
}

/** 返回 RAK 电源模块上报的母线电流，单位 mA（来自 IPSO 0xB8 DC_CURRENT） */
int16_t RAKSensorHub::getCurrentMa()
{
    return power.curMa;
}

/** 返回 RAK 电源模块上报的电池电量百分比 0..100（来自 IPSO 0xBA CAPACITY） */
int RAKSensorHub::getBusBatteryPercent()
{
    return (int)power.percent;
}

/** 根据电流是否大于 0 判断是否处于充电状态 */
bool RAKSensorHub::isCharging()
{
    return (power.curMa > 0) ? true : false;
}

/** 由 onewire_evt 在收到有效传感器数据时调用，用于更新 TelemetrySensor 的 lastRead 时间戳 */
void RAKSensorHub::setLastRead(uint32_t lastRead)
{
    this->lastRead = lastRead;
}

#endif // HAS_RAKHUB  
