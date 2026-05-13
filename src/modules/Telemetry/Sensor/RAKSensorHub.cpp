#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(HAS_RAKHUB)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "RAKSensorHub.h"
#include "TelemetrySensor.h"
#include "concurrency/LockGuard.h"
#include "concurrency/Periodic.h"
#include <RAK-OneWireSerial.h>
#include <onewire_master_protocol.h> // for DELIMTER/WAKEUPBYTE and frame layout

#include <cstring>
#include <set>
#if RAK_SENSORHUB_USB_PROFILE
#include <cstdlib>
#endif

using namespace concurrency;

#define BOOT_DATA_REQ

#ifndef RAK_SENSORHUB_DOWNLINK_POC
#define RAK_SENSORHUB_DOWNLINK_POC 0
#endif
#ifndef RAK_SENSORHUB_USB_PROFILE
#define RAK_SENSORHUB_USB_PROFILE 0
#endif

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

#if RAK_SENSORHUB_DOWNLINK_POC
static bool downlink_poc_pending = false;
static bool downlink_poc_done = false;
static uint8_t downlink_poc_pid = 0;
static uint8_t downlink_poc_step = 0;
// Two-phase POC:
// - clear phase: stop runtime polling + clear defaults, then request probe restart
// - config phase: apply template (IO_CFG + IO_ADDPOLLEX + IO_ENABLEPOLL)
static uint8_t downlink_poc_phase = 0; // 0=clear+restart, 1=config
static bool downlink_poc_wait_rejoin = false;
static uint32_t downlink_poc_wait_since = 0;
static uint8_t downlink_poc_wait_pid = 0;

// Downlink template that mirrors WisToolBox JSON IO_ADDPOLL fields.
// Goal: allow swapping sensor definitions without rewriting state machine code.
typedef struct {
    // 0: poll task (IO_ADDPOLLEX + IO_ENABLEPOLL), e.g. RS485 Modbus
    // 1: AIC decode mapping (IO_DECODE), for 4-20mA
    uint8_t taskKind;
    uint8_t taskId;
    uint32_t periodS;
    uint32_t timeoutMs;
    uint8_t retry;
    uint8_t datatype;
    float scale;
    uint16_t ipso;
    const char *profileName; // WisToolBox {PROFILE_NAME}, fixed 16 bytes on ProbeIO (e.g. "GE")
    const uint8_t *cmd;
    uint8_t cmdLen;
    // For IO_DECODE (AIC): engineering range and offset (core-1.2.27 com_if_rak_bank_decode_page_t)
    int32_t min;
    int32_t max;
    float offset;
} DownlinkPollTask;

typedef struct {
    const char *sensorName;
    uint8_t iface; // IOC_RS485, IOC_SDI12, ...
    // Minimal subset used by current POC; can be extended to IO_PSM/SNSR_CONF later.
    uint32_t baudrate;
    uint8_t databit;
    uint8_t stopbit;
    uint8_t parity;
    // Optional: IO_PSM (sent on IOC_CONTROL=9, funccode IO_PSM) for power management / warmup.
    bool usePsm;
    uint8_t psm_if_psm;
    uint8_t psm_psw;
    uint16_t psm_warmup_ms;
    uint8_t psm_pmode;
    uint8_t psm_method;
    const DownlinkPollTask *tasks;
    uint8_t taskCount;
} DownlinkSensorTemplate;

// JXBS-3001-EC template (aligned with WisToolBox JSON example).
static const uint8_t JXBS3001_CMD1[] = {0x01, 0x03, 0x00, 0x12, 0x00, 0x01};
static const uint8_t JXBS3001_CMD2[] = {0x01, 0x03, 0x00, 0x13, 0x00, 0x01};
static const uint8_t JXBS3001_CMD3[] = {0x01, 0x03, 0x00, 0x14, 0x00, 0x01};
static const uint8_t JXBS3001_CMD4[] = {0x01, 0x03, 0x00, 0x15, 0x00, 0x01};

static const DownlinkPollTask JXBS3001_EC_TASKS[] = {
    // TASK_ID1: ...0300120001:60:5000:2:6:0.1:112:{PROFILE_NAME}
    {.taskId = 1,
     .periodS = 60,
     .timeoutMs = 5000,
     .retry = 2,
     .datatype = 6,
     .scale = 0.1f,
     .ipso = 112,
     .profileName = "GE",
     .cmd = JXBS3001_CMD1,
     .cmdLen = sizeof(JXBS3001_CMD1)},
    // TASK_ID2: ...0300130001:60:5000:2:4:0.1:103:{PROFILE_NAME}
    {.taskId = 2,
     .periodS = 60,
     .timeoutMs = 5000,
     .retry = 2,
     .datatype = 4,
     .scale = 0.1f,
     .ipso = 103,
     .profileName = "GE",
     .cmd = JXBS3001_CMD2,
     .cmdLen = sizeof(JXBS3001_CMD2)},
    // TASK_ID3: ...0300140001:60:1000:2:6:1:19:{PROFILE_NAME}
    {.taskId = 3,
     .periodS = 60,
     .timeoutMs = 1000,
     .retry = 2,
     .datatype = 6,
     .scale = 1.0f,
     .ipso = 19,
     .profileName = "GE",
     .cmd = JXBS3001_CMD3,
     .cmdLen = sizeof(JXBS3001_CMD3)},
    // TASK_ID4: ...0300150001:60:1000:2:6:0.001:192:{PROFILE_NAME}
    {.taskId = 4,
     .periodS = 60,
     .timeoutMs = 1000,
     .retry = 2,
     .datatype = 6,
     .scale = 0.001f,
     .ipso = 192,
     .profileName = "GE",
     .cmd = JXBS3001_CMD4,
     .cmdLen = sizeof(JXBS3001_CMD4)},
};

static const DownlinkSensorTemplate JXBS3001_EC_TEMPLATE = {
    .sensorName = "JXBS-3001-EC",
    .iface = IOC_RS485,
    .baudrate = 9600,
    .databit = 8,
    .stopbit = 1,
    .parity = 0,
    .usePsm = false,
    .psm_if_psm = 0,
    .psm_psw = 0,
    .psm_warmup_ms = 0,
    .psm_pmode = 0,
    .psm_method = 0,
    .tasks = JXBS3001_EC_TASKS,
    .taskCount = (uint8_t)(sizeof(JXBS3001_EC_TASKS) / sizeof(JXBS3001_EC_TASKS[0])),
};

// SDSIN (Shandong Saien) RS485 soil 4-in-1: moisture/temp/EC/pH.
// Modbus holding regs (0-based): 0x0000 moisture(*10), 0x0001 temp(*10 signed), 0x0002 EC(uS/cm), 0x0003 pH(*10).
// We map them into IPSO: 0x70 (moisture), 0x67 (temperature), 0xC0 (EC), 0xC2 (pH).
static const uint8_t SDSIN4_CMD_MOIST[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
static const uint8_t SDSIN4_CMD_TEMP[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01};
static const uint8_t SDSIN4_CMD_EC[] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01};
static const uint8_t SDSIN4_CMD_PH[] = {0x01, 0x03, 0x00, 0x03, 0x00, 0x01};

static const DownlinkPollTask SDSIN_SOIL_4IN1_TASKS[] = {
    // moisture % (x10)
    {.taskId = 1,
     .periodS = 60,
     .timeoutMs = 5000,
     .retry = 2,
     .datatype = 6,
     .scale = 0.1f,
     .ipso = 112, // 0x70
     .profileName = "GE",
     .cmd = SDSIN4_CMD_MOIST,
     .cmdLen = sizeof(SDSIN4_CMD_MOIST)},
    // temperature C (x10, two's complement when <0)
    {.taskId = 2,
     .periodS = 60,
     .timeoutMs = 5000,
     .retry = 2,
     .datatype = 4,
     .scale = 0.1f,
     .ipso = 103, // 0x67
     .profileName = "GE",
     .cmd = SDSIN4_CMD_TEMP,
     .cmdLen = sizeof(SDSIN4_CMD_TEMP)},
    // EC (uS/cm). Use scale=0.001 so Meshtastic side reports in mS/cm (consistent with existing EC logging).
    {.taskId = 3,
     .periodS = 60,
     .timeoutMs = 5000,
     .retry = 2,
     .datatype = 6,
     .scale = 0.001f,
     .ipso = 192, // 0xC0
     .profileName = "GE",
     .cmd = SDSIN4_CMD_EC,
     .cmdLen = sizeof(SDSIN4_CMD_EC)},
    // pH (x10)
    {.taskId = 4,
     .periodS = 60,
     .timeoutMs = 5000,
     .retry = 2,
     .datatype = 6,
     .scale = 0.1f,
     .ipso = 194, // 0xC2
     .profileName = "GE",
     .cmd = SDSIN4_CMD_PH,
     .cmdLen = sizeof(SDSIN4_CMD_PH)},
};

static const DownlinkSensorTemplate SDSIN_SOIL_4IN1_TEMPLATE = {
    .sensorName = "SDSIN-Soil-4in1",
    .iface = IOC_RS485,
    .baudrate = 4800,
    .databit = 8,
    .stopbit = 1,
    .parity = 0,
    .usePsm = false,
    .psm_if_psm = 0,
    .psm_psw = 0,
    .psm_warmup_ms = 0,
    .psm_pmode = 0,
    .psm_method = 0,
    .tasks = SDSIN_SOIL_4IN1_TASKS,
    .taskCount = (uint8_t)(sizeof(SDSIN_SOIL_4IN1_TASKS) / sizeof(SDSIN_SOIL_4IN1_TASKS[0])),
};

// JXBS-4001-PH (RS485): template from WisToolBox JSON example.
// Task1: {DEV_ADDR}0300020001:60:1000:2:6:0.01:193:{PROFILE_NAME}
// IPSO=193 is 0xC1 (high-precision pH, 0.01 resolution).
static const uint8_t JXBS4001PH_CMD1[] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01};

static const DownlinkPollTask JXBS4001_PH_TASKS[] = {
    {.taskId = 1,
     .periodS = 60,
     .timeoutMs = 1000,
     .retry = 2,
     .datatype = 6,
     .scale = 0.01f,
     .ipso = 193, // 0xC1
     .profileName = "GE",
     .cmd = JXBS4001PH_CMD1,
     .cmdLen = sizeof(JXBS4001PH_CMD1)},
};

static const DownlinkSensorTemplate JXBS4001_PH_TEMPLATE = {
    .sensorName = "JXBS-4001-PH",
    .iface = IOC_RS485,
    .baudrate = 9600,
    .databit = 8,
    .stopbit = 1,
    .parity = 0,
    .usePsm = false,
    .psm_if_psm = 0,
    .psm_psw = 0,
    .psm_warmup_ms = 0,
    .psm_pmode = 0,
    .psm_method = 0,
    .tasks = JXBS4001_PH_TASKS,
    .taskCount = (uint8_t)(sizeof(JXBS4001_PH_TASKS) / sizeof(JXBS4001_PH_TASKS[0])),
};

// 4-20mA (AIC) template (core-1.2.27): uses IO_DECODE to map channel -> IPSO + min/max/offset + name.
// AIC periodic uploads are handled by ProbeIO rule engine; no IO_ADDPOLLEX/IO_ENABLEPOLL is needed.
static const DownlinkPollTask AIC_4_20MA_TASKS[] = {
    {.taskKind = 1,
     .taskId = 1, // AIC channel 1
     // WisToolBox example uses IPSO=130 (0x82). Keep it literal to match JSON.
     .ipso = 130,
     .profileName = "ULB16_05",
     .min = 0,
     .max = 5, // JSON: max=5 min=0 (engineering range)
     .offset = 0.0f},
};

static const DownlinkSensorTemplate AIC_4_20MA_TEMPLATE = {
    .sensorName = "AIC-4-20mA",
    .iface = IOC_AIC,
    .baudrate = 0,
    .databit = 0,
    .stopbit = 0,
    .parity = 0,
    // WisToolBox JSON:
    // atc+io_psm={PRB_ID}:9:1:10000:0:1  => if_psm=9 psw=1 warmup=10000 pmode=0 method=1
    .usePsm = true,
    .psm_if_psm = 9,
    .psm_psw = 1,
    .psm_warmup_ms = 10000,
    .psm_pmode = 0,
    .psm_method = 1,
    .tasks = AIC_4_20MA_TASKS,
    .taskCount = (uint8_t)(sizeof(AIC_4_20MA_TASKS) / sizeof(AIC_4_20MA_TASKS[0])),
};

#if RAK_SENSORHUB_DOWNLINK_POC && RAK_SENSORHUB_USB_PROFILE

// -----------------------------------------------------------------------------
// USB CDC text POC — replace compile-time profile without reflashing (same USB as LOG).
// IMPORTANT: Disconnect Meshtastic App / protobuf clients while sending lines (raw newline text).
// -----------------------------------------------------------------------------

static char rakhub_usb_line[280];
static size_t rakhub_usb_line_len = 0;

static bool rakhub_usb_override = false;       // When true, USB selection replaces RAK_SENSORHUB_DOWNLINK_TEMPLATE
static bool rakhub_usb_custom_active = false;  // Mutable RS485/AIC profile in rakhub_usb_*
static uint8_t rakhub_usb_builtin_id = 1;      // Used when override && !custom (0..4 same as macros)

static DownlinkPollTask rakhub_usb_custom_tasks[4];
static DownlinkSensorTemplate rakhub_usb_custom_tpl;
static uint8_t rakhub_usb_cmd[4][64];
static char rakhub_usb_name[4][17];
static char rakhub_usb_sensor_name[32] = "USB-profile";

static int rakhubHexNibble(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

static bool rakhubParseHexCmd(const char *hex, uint8_t *out, size_t cap, uint8_t *olen)
{
    const size_t hlen = strlen(hex);
    if (hlen < 2 || (hlen % 2u) != 0 || (hlen / 2u) > cap)
        return false;
    const size_t n = hlen / 2u;
    for (size_t i = 0; i < n; i++) {
        int hi = rakhubHexNibble(hex[i * 2]);
        int lo = rakhubHexNibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0)
            return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    *olen = (uint8_t)n;
    return true;
}

static void rakhubUsbTriggerDownlink(uint8_t pid)
{
    uint8_t p = pid;
    if (p == 0) {
        if (data_poll_pid_valid && provision_list.count(data_poll_pid))
            p = data_poll_pid;
        else if (provision_list.count(0x01))
            p = 0x01;
        else
            p = 0x01;
    }
    downlink_poc_done = false;
    downlink_poc_phase = 0;
    downlink_poc_step = 0;
    downlink_poc_wait_rejoin = false;
    downlink_poc_pending = true;
    downlink_poc_pid = p;
    LOG_INFO("RAKSensorHub USB: downlink re-armed PID=0x%02x", p);
}

#endif // RAK_SENSORHUB_DOWNLINK_POC && RAK_SENSORHUB_USB_PROFILE

#ifndef RAK_SENSORHUB_DOWNLINK_TEMPLATE
// 1: JXBS-3001-EC (existing), 2: SDSIN soil 4-in-1 (moisture/temp/EC/pH), 3: JXBS-4001-PH (pH only), 4: AIC 4-20mA
#define RAK_SENSORHUB_DOWNLINK_TEMPLATE 1
#endif

static const DownlinkSensorTemplate *getDownlinkTemplate()
{
#if RAK_SENSORHUB_DOWNLINK_POC && RAK_SENSORHUB_USB_PROFILE
    if (rakhub_usb_custom_active)
        return &rakhub_usb_custom_tpl;
    if (rakhub_usb_override) {
        switch (rakhub_usb_builtin_id) {
        case 0:
            // Clear-only: iface fields still needed for IOC_RMPOLL/RMPDEF; RS485 placeholder.
            return &JXBS3001_EC_TEMPLATE;
        case 2:
            return &SDSIN_SOIL_4IN1_TEMPLATE;
        case 3:
            return &JXBS4001_PH_TEMPLATE;
        case 4:
            return &AIC_4_20MA_TEMPLATE;
        case 1:
        default:
            return &JXBS3001_EC_TEMPLATE;
        }
    }
#endif
    // Initial POC uses a single fixed template; future work: select by provisioning info or external config.
    switch (RAK_SENSORHUB_DOWNLINK_TEMPLATE) {
    case 0:
        // Clear-only mode: template is unused (no IO_CFG/IO_ADDPOLLEX will be sent).
        return &JXBS3001_EC_TEMPLATE;
    case 2:
        return &SDSIN_SOIL_4IN1_TEMPLATE;
    case 3:
        return &JXBS4001_PH_TEMPLATE;
    case 4:
        return &AIC_4_20MA_TEMPLATE;
    case 1:
    default:
        return &JXBS3001_EC_TEMPLATE;
    }
}

static const char *iocFuncName(uint8_t funcode)
{
    switch (funcode) {
    case IO_CFG:
        return "IO_CFG";
    case IO_ADDPOLL:
        return "IO_ADDPOLL";
    case IO_ADDPOLLEX:
        return "IO_ADDPOLLEX";
    case IO_ENABLEPOLL:
        return "IO_ENABLEPOLL";
    case IO_POLLTASK:
        return "IO_POLLTASK";
    case IO_RMPDEF:
        return "IO_RMPDEF";
    case IO_RMPOLL:
        return "IO_RMPOLL";
    case IO_PSM:
        return "IO_PSM";
    case IO_CNT:
        return "IO_CNT";
    case IOPASSTHRH:
        return "IOPASSTHRH";
    default:
        return "UNKNOWN";
    }
}

static void scheduleDownlinkPoc(uint8_t pid)
{
    if (pid == 0 || pid == PID_MASTER || downlink_poc_pending)
        return;

    // If we were waiting for the probe to restart and rejoin, continue with config phase
    // only when the same PID re-appears.
    if (downlink_poc_wait_rejoin && pid == downlink_poc_wait_pid) {
        downlink_poc_phase = 1;
        downlink_poc_wait_rejoin = false;
        downlink_poc_pid = pid;
    } else {
        downlink_poc_phase = 0;
        downlink_poc_pid = pid;
    }
    downlink_poc_step = 0;
    downlink_poc_pending = true;
    LOG_INFO("RAKSensorHub downlink POC scheduled: PID=0x%02x", pid);
}

static bool sendNextDownlinkPoc()
{
    rak_ioc_polltask_frame_t polltask;
    const DownlinkSensorTemplate *tpl = getDownlinkTemplate();
#if RAK_SENSORHUB_USB_PROFILE
    const bool config_enabled = rakhub_usb_override ? (rakhub_usb_custom_active || (rakhub_usb_builtin_id != 0))
                                                     : (RAK_SENSORHUB_DOWNLINK_TEMPLATE != 0);
#else
    const bool config_enabled = (RAK_SENSORHUB_DOWNLINK_TEMPLATE != 0);
#endif

    if (!downlink_poc_pending || downlink_poc_done)
        return false;

    // Config phase layout:
    // 0: IO_CFG
    // 1..taskCount: IO_ADDPOLLEX per task
    // next taskCount: IO_ENABLEPOLL per task
    // final: IO_POLLTASK query
    //
    // Clear phase layout:
    // 0: IO_RMPOLL task=0 (stop runtime polling immediately)
    // 1: IO_RMPDEF portid=0 (clear defaults in flash)
    // 2: stop; wait for manual power-cycle and rejoin, then config phase starts
    if (downlink_poc_phase == 0) {
        if (downlink_poc_step == 0) {
            uint8_t taskid = 0;
            LOG_INFO("RAKSensorHub %s IOC TX: IO_RMPOLL stop all tasks", tpl->sensorName);
            RakSNHub_Protocl_API.ioc.send(downlink_poc_pid, IO_RMPOLL, tpl->iface, IOA_REQ, &taskid, sizeof(taskid));
        } else if (downlink_poc_step == 1) {
            LOG_INFO("RAKSensorHub %s IOC TX: IO_RMPDEF clear defaults", tpl->sensorName);
            RakSNHub_IOC_RmPollDef(downlink_poc_pid, tpl->iface, 0); // portid=0表示清除所有任务
        } else if (downlink_poc_step == 2) {
            downlink_poc_pending = false;
            if (!config_enabled) {
                downlink_poc_done = true;
                LOG_INFO("RAKSensorHub downlink POC clear-only done (RAK_SENSORHUB_DOWNLINK_TEMPLATE=0): PID=0x%02x",
                         downlink_poc_pid);
                return false;
            }
            downlink_poc_wait_rejoin = true;
            downlink_poc_wait_pid = downlink_poc_pid;
            downlink_poc_wait_since = millis();
            LOG_INFO("RAKSensorHub downlink POC cleared; power-cycle ProbeIO then wait for rejoin: PID=0x%02x", downlink_poc_pid);
            return false;
        } else {
            // Should not reach
            downlink_poc_pending = false;
            downlink_poc_done = true;
            return false;
        }
    } else if (downlink_poc_step == 0) {
        LOG_INFO("RAKSensorHub %s IOC TX: IO_CFG %s %lu:%u:%u:%u", tpl->sensorName,
                 (tpl->iface == IOC_RS485) ? "RS485" : "IF", (unsigned long)tpl->baudrate, (unsigned)tpl->databit,
                 (unsigned)tpl->stopbit, (unsigned)tpl->parity);
        if (tpl->iface == IOC_RS485) {
            RakSNHub_IOC_ConfigRS485(downlink_poc_pid, tpl->baudrate, tpl->databit, tpl->stopbit, tpl->parity);
        } else if (tpl->iface == IOC_AIC) {
            // AIC (4-20mA) uses IO_DECODE per channel; WisToolBox templates also send IO_PSM (IOC_CONTROL) first.
            if (tpl->usePsm) {
                struct __attribute__((packed)) ThisPsmFrame {
                    uint8_t if_psm;
                    uint8_t psw;
                    uint16_t warmuptime;
                    uint8_t pmode;
                    uint8_t method;
                } psm = {
                    .if_psm = tpl->psm_if_psm,
                    .psw = tpl->psm_psw,
                    .warmuptime = tpl->psm_warmup_ms,
                    .pmode = tpl->psm_pmode,
                    .method = tpl->psm_method,
                };
                LOG_INFO("RAKSensorHub %s IOC TX: IO_PSM if_psm=%u psw=%u warmup=%u pmode=%u method=%u",
                         tpl->sensorName, (unsigned)psm.if_psm, (unsigned)psm.psw, (unsigned)psm.warmuptime,
                         (unsigned)psm.pmode, (unsigned)psm.method);
                RakSNHub_Protocl_API.ioc.send(downlink_poc_pid, IO_PSM, IOC_CONTROL, IOA_REQ, (const uint8_t *)&psm,
                                              sizeof(psm));
            }
        }
    } else {
        if (tpl->iface == IOC_AIC) {
            // AIC config layout:
            // 1..taskCount: IO_DECODE per channel
            const uint8_t decodeStart = 1;
            const uint8_t doneStep = (uint8_t)(decodeStart + tpl->taskCount);
            if (downlink_poc_step >= decodeStart && downlink_poc_step < doneStep) {
                const uint8_t idx = (uint8_t)(downlink_poc_step - decodeStart);
                const DownlinkPollTask &t = tpl->tasks[idx];
                LOG_INFO("RAKSensorHub %s IOC TX: IO_DECODE(AIC) ch=%u IPSO=%u min=%ld max=%ld offset=%g name=%s",
                         tpl->sensorName, (unsigned)t.taskId, (unsigned)t.ipso, (long)t.min, (long)t.max, (double)t.offset,
                         (t.profileName != NULL) ? t.profileName : "");
                RakSNHub_IOC_DecodeAIC(downlink_poc_pid, t.taskId, (uint8_t)t.ipso, t.min, t.max, t.offset, t.profileName);
            } else {
                downlink_poc_pending = false;
                downlink_poc_done = true;
                LOG_INFO("RAKSensorHub %s downlink POC queued all IOC commands", tpl->sensorName);
                return false;
            }
        } else {
            const uint8_t addStart = 1;
            const uint8_t enableStart = (uint8_t)(addStart + tpl->taskCount);
            const uint8_t pollTaskStep = (uint8_t)(enableStart + tpl->taskCount);

            if (downlink_poc_step >= addStart && downlink_poc_step < enableStart) {
                const uint8_t idx = (uint8_t)(downlink_poc_step - addStart);
                const DownlinkPollTask &t = tpl->tasks[idx];
                LOG_INFO(
                    "RAKSensorHub %s IOC TX: IO_ADDPOLLEX task=%u cmdlen=%u period=%lu timeout=%lu retry=%u scale=%g "
                    "IPSO=%u datatype=%u name=%s",
                    tpl->sensorName, (unsigned)t.taskId, (unsigned)t.cmdLen, (unsigned long)t.periodS,
                    (unsigned long)t.timeoutMs, (unsigned)t.retry, (double)t.scale, (unsigned)t.ipso, (unsigned)t.datatype,
                    (t.profileName != NULL) ? t.profileName : "");
                RakSNHub_IOC_AddPollEx(downlink_poc_pid, tpl->iface, t.taskId, t.cmd, t.cmdLen, t.periodS, t.timeoutMs, t.retry,
                                       (uint8_t)t.ipso, t.scale, t.datatype, t.profileName);
            } else if (downlink_poc_step >= enableStart && downlink_poc_step < pollTaskStep) {
                const uint8_t idx = (uint8_t)(downlink_poc_step - enableStart);
                const DownlinkPollTask &t = tpl->tasks[idx];
                LOG_INFO("RAKSensorHub %s IOC TX: IO_ENABLEPOLL task=%u enable=1", tpl->sensorName, (unsigned)t.taskId);
                RakSNHub_IOC_EnablePoll(downlink_poc_pid, tpl->iface, t.taskId, 1);
            } else if (downlink_poc_step == pollTaskStep) {
                LOG_INFO("RAKSensorHub %s IOC TX: IO_POLLTASK query task=0", tpl->sensorName);
                polltask.taskid = 0;
                RakSNHub_Protocl_API.ioc.send(downlink_poc_pid, IO_POLLTASK, tpl->iface, IOA_RSP, (const uint8_t *)&polltask,
                                              sizeof(polltask));
            } else {
                downlink_poc_pending = false;
                downlink_poc_done = true;
                LOG_INFO("RAKSensorHub %s downlink POC queued all IOC commands", tpl->sensorName);
                return false;
            }
        }
    }

    downlink_poc_step++;
    return true;
}
#endif

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

static inline bool isAllZero(const uint8_t *p, uint16_t n)
{
    if (p == nullptr)
        return true;
    for (uint16_t i = 0; i < n; i++) {
        if (p[i] != 0)
            return false;
    }
    return true;
}

static bool parseDownlinkPocModbus(uint8_t taskId, uint8_t *msg, uint16_t len)
{
    const uint8_t *modbus = nullptr;
    uint8_t modbusLen = 0;

    if (len < 2 || msg[0] != RAK_IPSO_MODBUS)
        return false;

    // Many ProbeIO firmwares include a fixed 64-byte IPSO[F1] slot in get.data() responses even when no
    // IOC upload data is pending. That slot is often all-zero. Do not treat it as a link error.
    if (isAllZero(&msg[1], (uint16_t)(len - 1))) {
        return false;
    }

    /* Two possible payload layouts exist:
     * A) [F1][len][modbus...]
     * B) [F1][taskId][type=F1][datalen][modbus...]
     */
    if (len >= 5 && msg[2] == RAK_IPSO_MODBUS && msg[3] <= (len - 4)) {
        modbusLen = msg[3];
        modbus = &msg[4];
        taskId = msg[1];
        LOG_DEBUG("JXBS-3001-EC MODBUS raw task=%u len=%u head=%02x %02x %02x %02x %02x %02x %02x",
                  (unsigned)taskId, (unsigned)modbusLen,
                  (unsigned)modbus[0], (unsigned)modbus[1], (unsigned)modbus[2],
                  (unsigned)modbus[3], (unsigned)modbus[4], (unsigned)modbus[5], (unsigned)modbus[6]);
    } else if (len >= 3 && msg[1] <= (len - 2)) {
        modbusLen = msg[1];
        modbus = &msg[2];
        LOG_DEBUG("JXBS-3001-EC MODBUS raw task=%u len=%u head=%02x %02x %02x %02x %02x %02x %02x",
                  (unsigned)taskId, (unsigned)modbusLen,
                  (unsigned)modbus[0], (unsigned)modbus[1], (unsigned)modbus[2],
                  (unsigned)modbus[3], (unsigned)modbus[4], (unsigned)modbus[5], (unsigned)modbus[6]);
    } else {
        modbusLen = (uint8_t)(len - 1);
        modbus = &msg[1];
    }

    // If the "slot" is present but empty, skip quietly (avoid log spam during periodic get.data polls).
    if (modbusLen == 0) {
        return false;
    }

    if (modbusLen < 7 || modbus[1] != 0x03 || modbus[2] < 2) {
        LOG_INFO("JXBS-3001-EC MODBUS task=%u invalid response len=%u", (unsigned)taskId, (unsigned)modbusLen);
        return false;
    }

    uint16_t raw = ((uint16_t)modbus[3] << 8) | modbus[4];
    uint32_t now = millis();

    switch (taskId) {
    case 0: // Pass-through immediate read, use the same mapping as task 1 for link validation.
    case 1: { // JXBS-3001-EC WaterContent / IPSO 0x70, scale 0.1 %
        float water = raw / 10.0f;
        if (water >= 0.0f && water <= 100.0f) {
            setScalar(env.high_precision_humidity, water, now);
            LOG_INFO("JXBS-3001-EC WaterContent(task=%u): raw=%u, %.1f %%", (unsigned)taskId, (unsigned)raw, water);
            return true;
        }
        break;
    }
    case 2: { // Temperature, scale 0.1 C
        int16_t signedRaw = (int16_t)raw;
        float temperature = signedRaw / 10.0f;
        if (temperature >= -50.0f && temperature <= 130.0f) {
            setScalar(env.temperature, temperature, now);
            LOG_INFO("JXBS-3001-EC Temperature(task=2): raw=%d, %.1f C", (int)signedRaw, temperature);
            return true;
        }
        break;
    }
    case 3: { // Salinity, scale 1 mg/L
        setScalar(env.salinity, (float)raw, now);
        LOG_INFO("JXBS-3001-EC Salinity(task=3): raw=%u, %.0f mg/L", (unsigned)raw, env.salinity.value);
        return true;
    }
    case 4: { // Conductivity, scale 0.001 mS/cm.
        float ec_ms = raw / 1000.0f;
        setScalar(env.ec, ec_ms, now);
        LOG_INFO("JXBS-3001-EC Conductivity(task=4): raw=%u, %.3f mS/cm", (unsigned)raw, ec_ms);
        return true;
    }
    default:
        LOG_INFO("JXBS-3001-EC MODBUS task=%u not mapped", (unsigned)taskId);
        return false;
    }

    LOG_INFO("JXBS-3001-EC MODBUS task=%u raw=%u out of range", (unsigned)taskId, (unsigned)raw);
    return false;
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
        if (last_sent_pid_valid && pid == last_sent_pid && pid != PID_MASTER && provision_list.count(pid) == 0) {
            provision_list.insert(pid);
            data_poll_pid = pid;
            data_poll_pid_valid = true;
            LOG_INFO("+ADD:PID:[%02x] from discovery response", pid);
#if RAK_SENSORHUB_DOWNLINK_POC
            scheduleDownlinkPoc(pid);
#endif
        }
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
#if RAK_SENSORHUB_DOWNLINK_POC
        scheduleDownlinkPoc(msg[0]);
#endif
        break;

#if RAK_SENSORHUB_DOWNLINK_POC
    case SNHUBAPI_EVT_IOC_RSP: {
        RakSNHub_IOC_Rsp_t rsp;
        if (RakSNHub_IOC_ParseRsp(msg, len, &rsp) == RET_OK) {
            LOG_INFO("RAKSensorHub IOC RSP: func=%s(0x%02x) iface=0x%02x action=0x%02x data_len=%u", iocFuncName(rsp.funcode),
                     rsp.funcode, rsp.iface, rsp.action, rsp.data_len);
        } else {
            LOG_INFO("RAKSensorHub IOC RSP: invalid len=%u", len);
        }
        break;
    }
#endif

    case SNHUBAPI_EVT_GET_INTV:
        break;

    case SNHUBAPI_EVT_GET_ENABLE:
        LOG_INFO("+EVT:PID[%02x],ENABLE[%02x]", pid, msg[0]);
        break;

    case SNHUBAPI_EVT_SDATA_REQ:  // IPSO parse from sensor data request response
        LOG_INFO("+EVT:PID[%02x],IPSO[%02x]", pid, msg[0]);
        if (msg[0] == 0x82 || msg[0] == RAK_IPSO_ANALOG_INPUT) {
            char hex[160] = {0};
            size_t o = 0;
            for (uint16_t i = 0; i < len && o + 3 < sizeof(hex); i++) {
                o += (size_t)snprintf(&hex[o], sizeof(hex) - o, "%02X ", (unsigned)msg[i]);
            }
            LOG_INFO("IPSO[%02x] raw(len=%u): %s", (unsigned)msg[0], (unsigned)len, hex);
        }
        switch (msg[0]) {
        case RAK_IPSO_MODBUS: {
            if (parseDownlinkPocModbus(sid, msg, len))
                rakSensorHub.setLastRead(millis());
            break;
        }
        case 0x82: { // Custom IPSO used by some AIC templates (e.g. WisToolBox: io_decode ... IPSO=130)
            const uint32_t now = millis();
            if (len >= 5) {
                uint32_t raw = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8) | ((uint32_t)msg[3] << 16) | ((uint32_t)msg[4] << 24);
                LOG_INFO("AIC IPSO[0x82] raw32=%lu (0x%08lx)", (unsigned long)raw, (unsigned long)raw);
                // Empirically (core-1.2.27 AIC + IPSO mult), 0x82 is often "mm" for water level / distance.
                // Meshtastic EnvironmentMetrics has a dedicated 'distance' field in mm.
                setScalar(env.distance, (float)raw, now);
            } else if (len >= 3) {
                uint16_t raw16 = ((uint16_t)msg[2] << 8) | msg[1];
                LOG_INFO("AIC IPSO[0x82] raw16=%u (0x%04x)", (unsigned)raw16, (unsigned)raw16);
                setScalar(env.distance, (float)raw16, now);
            }
            break;
        }
        case RAK_IPSO_ANALOG_INPUT: { // 0x02 standard analog input (2 bytes, little-endian)
            if (len < 3)
                break;
            uint16_t raw16 = ((uint16_t)msg[2] << 8) | msg[1];
            LOG_INFO("Analog input IPSO[0x02] raw=%u", (unsigned)raw16);
            break;
        }
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
            // RAK2560 Sensor Data Format V0.11: IPSO 0x68 is 1 %RH per bit (0..100).
            float humidity = (float)hum_raw;
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
            setScalar(env.high_precision_humidity, moisture, millis());
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
            // Do not overwrite with 0 (sensor may send 0 for "no data" or second channel)
            if (co2 > 0.0f) {
                setScalar(env.co2, co2, millis());
                LOG_INFO("CO2 sensor: %.2f ppm", co2);
            }
            break;
        }
        case RAK_IPSO_HP_EC: { // High-precision EC (0x7F), 4 bytes little-endian
            if (len < 5)
                break;
            uint32_t raw = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8) | ((uint32_t)msg[3] << 16) | ((uint32_t)msg[4] << 24);
            // Keep same unit convention as existing EC handler: mS/cm (raw scaled by 1000).
            if (raw == 0 && env.high_precision_ec.valid) {
                LOG_INFO("High precision EC (0x7F): raw=0 (skip overwrite)");
                break;
            }
            // RAK2560 Sensor Data Format V0.11: IPSO 0x7F is 0.001 uS/cm per bit.
            // Convert to mS/cm: (raw * 0.001 uS/cm) / 1000 = raw / 1,000,000.
            float ec_ms = raw / 1000000.0f;
            setScalar(env.high_precision_ec, ec_ms, millis());
            // Also populate env.ec so any existing consumers see EC even when only 0x7F is emitted.
            setScalar(env.ec, ec_ms, millis());
            LOG_INFO("High precision EC (0x7F): raw=%lu, %.3f (mS/cm units)", (unsigned long)raw, ec_ms);
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
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            // RK200-03 sample indicates factor=1 => raw is W/m² directly
            float rad = (float)raw;
            // User-provided valid range: 0..2000 W/m²
            if (rad < 0.0f || rad > 2000.0f) {
                LOG_INFO("Ignore pyranometer radiation value out of range: %.1f W/m2", rad);
                break;
            }
            setScalar(env.radiation, rad, millis());
            LOG_INFO("Pyranometer radiation: %.1f", env.radiation.value);
            break;
        }
        // RAK9154 / power module mapping (per RAK docs and existing RAK9154Sensor):
        // - 0xB8 (RAK_IPSO_CAPACITY): Battery SOC %, 1 byte 0..100
        // - 0xB9 (RAK_IPSO_DC_CURRENT): Battery current, raw * 0.01 A
        // - 0xBA (RAK_IPSO_DC_VOLTAGE): Battery voltage, raw * 0.01 V
        case RAK_IPSO_CAPACITY: { // 0xB8 Battery SOC 0..100 %
            if (len < 2)
                break;
            power.percent = msg[1];
            if (power.percent > 100)
                power.percent = 100;
            LOG_INFO("Battery capacity: %u %%", (unsigned)power.percent);
            break;
        }
        case RAK_IPSO_DC_CURRENT: { // 0xB9 Battery current raw * 0.01 A (unsigned; no negative display)
            if (len < 3)
                break;
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            float amps = raw * 0.01f;
            if (amps > 50.0f) {
                LOG_INFO("Ignore battery current out of range: %.3f A", amps);
                break;
            }
            power.curMa = (int16_t)(amps * 1000.0f);
            LOG_INFO("Battery current: %.3f A", (float)power.curMa / 1000.0f);
            break;
        }
        case RAK_IPSO_DC_VOLTAGE: { // 0xBA Battery voltage raw * 0.01 V
            if (len < 3)
                break;
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            float volts = raw * 0.01f;
            // RAK9154 output spec: approx 9..13.2 V; accept a safe wider band and drop spikes
            if (volts < 0.0f || volts > 20.0f) {
                LOG_INFO("Ignore battery voltage out of range: %.2f V", volts);
                break;
            }
            power.volMv = (uint16_t)(volts * 1000.0f);
            LOG_INFO("Battery voltage: %.2f V", volts);
            break;
        }
        case RAK_IPSO_HP_PH: {  // High-precision pH (0xC1), 0.01 resolution
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            float ph = raw / 100.0f;
            setScalar(env.soil_ph, ph, millis());
            LOG_INFO("Soil pH: %.2f", ph);
            break;
        }
        case RAK_IPSO_PH: {  // pH (0xC2), 0.1 resolution (per RAK sensor data format)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            // 0.1 pH per bit (e.g. raw=70 => pH=7.0)
            if (raw == 0 && env.ph.valid) {
                LOG_INFO("pH (0xC2): raw=0 (skip overwrite)");
                break;
            }
            float ph = raw / 10.0f;
            if (ph < 0.0f || ph > 14.0f) {
                LOG_INFO("Ignore pH value out of range: %.2f", ph);
                break;
            }
            setScalar(env.ph, ph, millis());
            LOG_INFO("pH: %.2f", ph);
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
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            // Some probes report 0 when the channel is not available; avoid overwriting a valid reading with 0.
            if (raw == 0 && env.salinity.valid) {
                LOG_INFO("Salinity: raw=0 (skip overwrite)");
                break;
            }
            // Per sensor spec: 1 mg/L per bit, range 0..65535 mg/L.
            setScalar(env.salinity, (float)raw, millis());
            LOG_INFO("Salinity: %u mg/L", (unsigned)raw);
            break;
        }
        case RAK_IPSO_EC: {  // Conductivity EC (0xC0), 0.001 mS/cm (0.001 µS/cm per bit)
            if (len < 3)
                break;
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            // Some probes report 0 when the channel is not available; avoid overwriting a valid reading with 0.
            if (raw == 0 && env.ec.valid) {
                LOG_INFO("EC: raw=0 (skip overwrite)");
                break;
            }
            // Keep same unit convention as existing EC handler: mS/cm (raw scaled by 1000).
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
        if (msg[0] == 0x82 || msg[0] == RAK_IPSO_ANALOG_INPUT) {
            char hex[160] = {0};
            size_t o = 0;
            for (uint16_t i = 0; i < len && o + 3 < sizeof(hex); i++) {
                o += (size_t)snprintf(&hex[o], sizeof(hex) - o, "%02X ", (unsigned)msg[i]);
            }
            LOG_INFO("REPORT IPSO[%02x] raw(len=%u): %s", (unsigned)msg[0], (unsigned)len, hex);
        }

        switch (msg[0]) {
        case RAK_IPSO_MODBUS: {
            if (parseDownlinkPocModbus(sid, msg, len))
                rakSensorHub.setLastRead(millis());
            break;
        }
        case 0x82: { // Custom IPSO used by some AIC templates (e.g. WisToolBox: io_decode ... IPSO=130)
            const uint32_t now = millis();
            if (len >= 5) {
                uint32_t raw = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8) | ((uint32_t)msg[3] << 16) | ((uint32_t)msg[4] << 24);
                LOG_INFO("AIC REPORT IPSO[0x82] raw32=%lu (0x%08lx)", (unsigned long)raw, (unsigned long)raw);
                setScalar(env.distance, (float)raw, now);
            } else if (len >= 3) {
                uint16_t raw16 = ((uint16_t)msg[2] << 8) | msg[1];
                LOG_INFO("AIC REPORT IPSO[0x82] raw16=%u (0x%04x)", (unsigned)raw16, (unsigned)raw16);
                setScalar(env.distance, (float)raw16, now);
            }
            break;
        }
        case RAK_IPSO_ANALOG_INPUT: { // 0x02 standard analog input (2 bytes, little-endian)
            if (len < 3)
                break;
            uint16_t raw16 = ((uint16_t)msg[2] << 8) | msg[1];
            LOG_INFO("Analog input REPORT IPSO[0x02] raw=%u", (unsigned)raw16);
            break;
        }
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
            // RAK2560 Sensor Data Format V0.11: IPSO 0x68 is 1 %RH per bit (0..100).
            float humidity = (float)hum_raw;
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
            setScalar(env.high_precision_humidity, moisture, millis());
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
            if (co2 > 0.0f) {
                setScalar(env.co2, co2, millis());
                LOG_INFO("CO2 sensor: %.2f ppm", co2);
            }
            break;
        }
        case RAK_IPSO_HP_EC: { // High-precision EC (0x7F), 4 bytes little-endian
            if (len < 5)
                break;
            uint32_t raw = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8) | ((uint32_t)msg[3] << 16) | ((uint32_t)msg[4] << 24);
            if (raw == 0 && env.high_precision_ec.valid) {
                LOG_INFO("High precision EC (0x7F): raw=0 (skip overwrite)");
                break;
            }
            float ec_ms = raw / 1000000.0f;
            setScalar(env.high_precision_ec, ec_ms, millis());
            setScalar(env.ec, ec_ms, millis());
            LOG_INFO("High precision EC (0x7F): raw=%lu, %.3f (mS/cm units)", (unsigned long)raw, ec_ms);
            break;
        }
        case RAK_IPSO_CAPACITY: { // 0xB8 Battery SOC 0..100 %
            if (len < 2)
                break;
            power.percent = msg[1];
            if (power.percent > 100)
                power.percent = 100;
            LOG_INFO("Battery capacity: %u %%", (unsigned)power.percent);
            break;
        }
        case RAK_IPSO_DC_CURRENT: { // 0xB9 Battery current raw * 0.01 A (unsigned; no negative display)
            if (len < 3)
                break;
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            float amps = raw * 0.01f;
            if (amps > 50.0f) {
                LOG_INFO("Ignore battery current out of range: %.3f A", amps);
                break;
            }
            power.curMa = (int16_t)(amps * 1000.0f);
            LOG_INFO("Battery current: %.3f A", (float)power.curMa / 1000.0f);
            break;
        }
        case RAK_IPSO_DC_VOLTAGE: { // 0xBA Battery voltage raw * 0.01 V
            if (len < 3)
                break;
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            float volts = raw * 0.01f;
            // RAK9154 output spec: approx 9..13.2 V; accept a safe wider band and drop spikes
            if (volts < 0.0f || volts > 20.0f) {
                LOG_INFO("Ignore battery voltage out of range: %.2f V", volts);
                break;
            }
            power.volMv = (uint16_t)(volts * 1000.0f);
            LOG_INFO("Battery voltage: %.2f V", volts);
            break;
        }
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
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            float rad = (float)raw;
            if (rad < 0.0f || rad > 2000.0f) {
                LOG_INFO("Ignore pyranometer radiation value out of range: %.1f W/m2", rad);
                break;
            }
            setScalar(env.radiation, rad, millis());
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
        case RAK_IPSO_PH: {  // pH (0xC2)
            if (len < 3)
                break;
            int16_t raw = (msg[2] << 8) + msg[1];
            if (raw == 0 && env.ph.valid) {
                LOG_INFO("pH (0xC2): raw=0 (skip overwrite)");
                break;
            }
            float ph = raw / 10.0f;
            if (ph < 0.0f || ph > 14.0f) {
                LOG_INFO("Ignore pH value out of range: %.2f", ph);
                break;
            }
            setScalar(env.ph, ph, millis());
            LOG_INFO("pH: %.2f", ph);
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
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            if (raw == 0 && env.salinity.valid) {
                LOG_INFO("Salinity: raw=0 (skip overwrite)");
                break;
            }
            setScalar(env.salinity, (float)raw, millis());
            LOG_INFO("Salinity: %u mg/L", (unsigned)raw);
            break;
        }
        case RAK_IPSO_EC: {  // Conductivity EC (0xC0), 0.001 mS/cm (0.001 µS/cm per bit)
            if (len < 3)
                break;
            uint16_t raw = (uint16_t)((msg[2] << 8) + msg[1]);
            if (raw == 0 && env.ec.valid) {
                LOG_INFO("EC: raw=0 (skip overwrite)");
                break;
            }
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
    concurrency::LockGuard guard(&onewireLock);


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
        // Frame format (per RAK docs): start(0xFF), lenL, lenH, type(0x45/0x4D), flag(0x02), ...
        // PID at buff[34]; some probes use buff[33]=0xFF. 0xFF = broadcast -> assign next free PID.
        if (total_needed >= 36 && (buff[3] == 0x45 || buff[3] == 0x4D)) {
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
#if RAK_SENSORHUB_DOWNLINK_POC
                scheduleDownlinkPoc(pid);
#endif
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

#if RAK_SENSORHUB_DOWNLINK_POC && RAK_SENSORHUB_USB_PROFILE

static void rakhubUsbHandleLine(char *line)
{
    while (*line == ' ' || *line == '\t')
        line++;

    char *cr = strchr(line, '\r');
    if (cr)
        *cr = '\0';

    if (strncmp(line, "RAKHUB", 6) != 0)
        return;

    char *cursor = line + 6;
    while (*cursor == ' ' || *cursor == '\t')
        cursor++;

    char *cmd = cursor;
    char *rest = strchr(cursor, ' ');
    if (rest) {
        *rest++ = '\0';
        while (*rest == ' ' || *rest == '\t')
            rest++;
    } else {
        rest = strchr(cmd, '\0');
    }

    if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        LOG_INFO(
            "RAKHUB HELP: Disconnect protobuf app first. LF-terminated lines:\nCOMPILE — use flash macro\tBUILTIN "
            "<0-4> — built-in preset\nRS485 baud=9600 databit=8 stop=1 parity=0 task=1 hex=010300120001 "
            "period=60 timeout=5000 retry=2 scale=0.1 ipso=112 dtype=6 name=GE [slot=0..3 psm=0]"
            "\nAIC ch=1 ipso=130 min=0 max=5 off=0 name=XXX [psm=1 if_psm=9 psw=1 warmup=10000 pmode=0 method=1]\nAPPLY [pid_hex] "
            "— ARM downlink POC\nSTATUS");
        return;
    }

    if (strcmp(cmd, "STATUS") == 0) {
        LOG_INFO("RAKHUB STATUS: usb_override=%d custom=%d builtin_id=%u compile_tpl=%d pending_poc=%d done=%d phase=%u",
                 (int)rakhub_usb_override, (int)rakhub_usb_custom_active, (unsigned)rakhub_usb_builtin_id,
                 RAK_SENSORHUB_DOWNLINK_TEMPLATE, (int)downlink_poc_pending, (int)downlink_poc_done,
                 (unsigned)downlink_poc_phase);
        return;
    }

    if (strcmp(cmd, "COMPILE") == 0) {
        rakhub_usb_override = false;
        rakhub_usb_custom_active = false;
        LOG_INFO("RAKHUB: using compile-time RAK_SENSORHUB_DOWNLINK_TEMPLATE=%d", RAK_SENSORHUB_DOWNLINK_TEMPLATE);
        return;
    }

    if (strcmp(cmd, "BUILTIN") == 0) {
        unsigned long n = strtoul(rest, nullptr, 10);
        if (n > 4) {
            LOG_INFO("RAKHUB BUILTIN: id must be 0..4");
            return;
        }
        rakhub_usb_override = true;
        rakhub_usb_custom_active = false;
        rakhub_usb_builtin_id = (uint8_t)n;
        LOG_INFO("RAKHUB BUILTIN=%u stored (clear-only if 0). Send RAKHUB APPLY.", (unsigned)rakhub_usb_builtin_id);
        return;
    }

    if (strcmp(cmd, "APPLY") == 0) {
        uint8_t pid = 0;
        if (*rest)
            pid = (uint8_t)strtoul(rest, nullptr, 16);
        rakhubUsbTriggerDownlink(pid);
        return;
    }

    if (strcmp(cmd, "RS485") == 0) {
        uint32_t baud = 9600;
        uint8_t databit = 8, stopbit = 1, parity = 0;
        uint8_t slot = 0;
        bool has_slot = false;
        uint8_t taskid = 1;
        uint32_t periodS = 60, timeoutMs = 5000;
        uint8_t retry = 2;
        uint8_t datatype = 6;
        float scale = 0.1f;
        uint16_t ipso = 112;
        char hexbuf[128] = {0};
        char namebuf[17] = {0};
        strncpy(namebuf, "GE", sizeof(namebuf) - 1);
        bool has_hex = false;
        bool usePsm = false;
        uint8_t psm_if = 0, psm_psw = 0, psm_pmode = 0, psm_method = 0;
        uint16_t psm_warmup = 0;

        char kvbuf[232];
        strncpy(kvbuf, rest, sizeof(kvbuf) - 1);
        kvbuf[sizeof(kvbuf) - 1] = '\0';

        for (char *tok = strtok(kvbuf, " \t"); tok != nullptr; tok = strtok(nullptr, " \t")) {
            char *eq = strchr(tok, '=');
            if (!eq)
                continue;
            *eq++ = '\0';
            if (strcmp(tok, "baud") == 0)
                baud = (uint32_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "databit") == 0)
                databit = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "stop") == 0)
                stopbit = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "parity") == 0)
                parity = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "slot") == 0) {
                slot = (uint8_t)strtoul(eq, nullptr, 10);
                has_slot = true;
            } else if (strcmp(tok, "task") == 0)
                taskid = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "hex") == 0) {
                strncpy(hexbuf, eq, sizeof(hexbuf) - 1);
                has_hex = true;
            } else if (strcmp(tok, "period") == 0)
                periodS = (uint32_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "timeout") == 0)
                timeoutMs = (uint32_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "retry") == 0)
                retry = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "scale") == 0)
                scale = strtof(eq, nullptr);
            else if (strcmp(tok, "ipso") == 0)
                ipso = (uint16_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "dtype") == 0)
                datatype = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "name") == 0) {
                strncpy(namebuf, eq, sizeof(namebuf) - 1);
                namebuf[sizeof(namebuf) - 1] = '\0';
            } else if (strcmp(tok, "psm") == 0)
                usePsm = strtoul(eq, nullptr, 10) != 0;
            else if (strcmp(tok, "if_psm") == 0)
                psm_if = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "psw") == 0)
                psm_psw = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "warmup") == 0)
                psm_warmup = (uint16_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "pmode") == 0)
                psm_pmode = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "method") == 0)
                psm_method = (uint8_t)strtoul(eq, nullptr, 10);
        }

        if (!has_hex || strlen(hexbuf) < 2) {
            LOG_INFO("RAKHUB RS485: missing hex=");
            return;
        }
        if (slot >= (sizeof(rakhub_usb_custom_tasks) / sizeof(rakhub_usb_custom_tasks[0]))) {
            LOG_INFO("RAKHUB RS485: slot must be 0..3");
            return;
        }
        uint8_t cmdlen = 0;
        memset(rakhub_usb_cmd[slot], 0, sizeof(rakhub_usb_cmd[slot]));
        if (!rakhubParseHexCmd(hexbuf, rakhub_usb_cmd[slot], sizeof(rakhub_usb_cmd[slot]), &cmdlen)) {
            LOG_INFO("RAKHUB RS485: invalid hex string");
            return;
        }

        if (!has_slot || slot == 0) {
            memset(rakhub_usb_custom_tasks, 0, sizeof(rakhub_usb_custom_tasks));
            memset(rakhub_usb_name, 0, sizeof(rakhub_usb_name));
        }

        memset(rakhub_usb_name[slot], 0, sizeof(rakhub_usb_name[slot]));
        strncpy(rakhub_usb_name[slot], namebuf, 16);

        memset(&rakhub_usb_custom_tasks[slot], 0, sizeof(rakhub_usb_custom_tasks[slot]));
        rakhub_usb_custom_tasks[slot].taskKind = 0;
        rakhub_usb_custom_tasks[slot].taskId = taskid;
        rakhub_usb_custom_tasks[slot].periodS = periodS;
        rakhub_usb_custom_tasks[slot].timeoutMs = timeoutMs;
        rakhub_usb_custom_tasks[slot].retry = retry;
        rakhub_usb_custom_tasks[slot].datatype = datatype;
        rakhub_usb_custom_tasks[slot].scale = scale;
        rakhub_usb_custom_tasks[slot].ipso = ipso;
        rakhub_usb_custom_tasks[slot].profileName = rakhub_usb_name[slot];
        rakhub_usb_custom_tasks[slot].cmd = rakhub_usb_cmd[slot];
        rakhub_usb_custom_tasks[slot].cmdLen = cmdlen;

        strncpy(rakhub_usb_sensor_name, "USB-RS485", sizeof(rakhub_usb_sensor_name) - 1);
        rakhub_usb_sensor_name[sizeof(rakhub_usb_sensor_name) - 1] = '\0';
        memset(&rakhub_usb_custom_tpl, 0, sizeof(rakhub_usb_custom_tpl));
        rakhub_usb_custom_tpl.sensorName = rakhub_usb_sensor_name;
        rakhub_usb_custom_tpl.iface = IOC_RS485;
        rakhub_usb_custom_tpl.baudrate = baud;
        rakhub_usb_custom_tpl.databit = databit;
        rakhub_usb_custom_tpl.stopbit = stopbit;
        rakhub_usb_custom_tpl.parity = parity;
        rakhub_usb_custom_tpl.usePsm = usePsm;
        rakhub_usb_custom_tpl.psm_if_psm = psm_if;
        rakhub_usb_custom_tpl.psm_psw = psm_psw;
        rakhub_usb_custom_tpl.psm_warmup_ms = psm_warmup;
        rakhub_usb_custom_tpl.psm_pmode = psm_pmode;
        rakhub_usb_custom_tpl.psm_method = psm_method;
        rakhub_usb_custom_tpl.tasks = rakhub_usb_custom_tasks;
        rakhub_usb_custom_tpl.taskCount = has_slot ? (uint8_t)(slot + 1) : 1;

        rakhub_usb_override = true;
        rakhub_usb_custom_active = true;
        LOG_INFO("RAKHUB RS485 slot=%u task=%u stored (%lu baud, cmdlen=%u, tasks=%u). Send RAKHUB APPLY.",
                 (unsigned)slot, (unsigned)taskid, (unsigned long)baud, (unsigned)cmdlen,
                 (unsigned)rakhub_usb_custom_tpl.taskCount);
        return;
    }

    if (strcmp(cmd, "AIC") == 0) {
        uint8_t ch = 1;
        uint16_t ipso_ai = 130;
        int32_t min_v = 0, max_v = 5;
        float offset_v = 0.0f;
        char namebuf[17] = {0};
        strncpy(namebuf, "ULB16_05", sizeof(namebuf) - 1);
        bool usePsm = true;
        uint8_t psm_if = 9, psm_psw = 1, psm_pmode = 0, psm_method = 1;
        uint16_t psm_warmup = 10000;

        char kvbuf[232];
        strncpy(kvbuf, rest, sizeof(kvbuf) - 1);
        kvbuf[sizeof(kvbuf) - 1] = '\0';

        for (char *tok = strtok(kvbuf, " \t"); tok != nullptr; tok = strtok(nullptr, " \t")) {
            char *eq = strchr(tok, '=');
            if (!eq)
                continue;
            *eq++ = '\0';
            if (strcmp(tok, "ch") == 0)
                ch = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "ipso") == 0)
                ipso_ai = (uint16_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "min") == 0)
                min_v = (int32_t)strtol(eq, nullptr, 10);
            else if (strcmp(tok, "max") == 0)
                max_v = (int32_t)strtol(eq, nullptr, 10);
            else if (strcmp(tok, "off") == 0)
                offset_v = strtof(eq, nullptr);
            else if (strcmp(tok, "name") == 0) {
                strncpy(namebuf, eq, sizeof(namebuf) - 1);
                namebuf[sizeof(namebuf) - 1] = '\0';
            } else if (strcmp(tok, "psm") == 0)
                usePsm = strtoul(eq, nullptr, 10) != 0;
            else if (strcmp(tok, "if_psm") == 0)
                psm_if = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "psw") == 0)
                psm_psw = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "warmup") == 0)
                psm_warmup = (uint16_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "pmode") == 0)
                psm_pmode = (uint8_t)strtoul(eq, nullptr, 10);
            else if (strcmp(tok, "method") == 0)
                psm_method = (uint8_t)strtoul(eq, nullptr, 10);
        }

        memset(rakhub_usb_name[0], 0, sizeof(rakhub_usb_name[0]));
        strncpy(rakhub_usb_name[0], namebuf, 16);

        memset(&rakhub_usb_custom_tasks[0], 0, sizeof(rakhub_usb_custom_tasks[0]));
        rakhub_usb_custom_tasks[0].taskKind = 1;
        rakhub_usb_custom_tasks[0].taskId = ch;
        rakhub_usb_custom_tasks[0].ipso = ipso_ai;
        rakhub_usb_custom_tasks[0].profileName = rakhub_usb_name[0];
        rakhub_usb_custom_tasks[0].min = min_v;
        rakhub_usb_custom_tasks[0].max = max_v;
        rakhub_usb_custom_tasks[0].offset = offset_v;

        strncpy(rakhub_usb_sensor_name, "USB-AIC", sizeof(rakhub_usb_sensor_name) - 1);
        rakhub_usb_sensor_name[sizeof(rakhub_usb_sensor_name) - 1] = '\0';
        memset(&rakhub_usb_custom_tpl, 0, sizeof(rakhub_usb_custom_tpl));
        rakhub_usb_custom_tpl.sensorName = rakhub_usb_sensor_name;
        rakhub_usb_custom_tpl.iface = IOC_AIC;
        rakhub_usb_custom_tpl.usePsm = usePsm;
        rakhub_usb_custom_tpl.psm_if_psm = psm_if;
        rakhub_usb_custom_tpl.psm_psw = psm_psw;
        rakhub_usb_custom_tpl.psm_warmup_ms = psm_warmup;
        rakhub_usb_custom_tpl.psm_pmode = psm_pmode;
        rakhub_usb_custom_tpl.psm_method = psm_method;
        rakhub_usb_custom_tpl.tasks = rakhub_usb_custom_tasks;
        rakhub_usb_custom_tpl.taskCount = 1;

        rakhub_usb_override = true;
        rakhub_usb_custom_active = true;
        LOG_INFO("RAKHUB AIC profile stored (ch=%u). Send RAKHUB APPLY.", (unsigned)ch);
        return;
    }

    LOG_INFO("RAKHUB: unknown \"%s\". Try RAKHUB HELP.", cmd);
}

static void rakhubUsbPollSerial()
{
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0)
            break;
        if (c == '\r')
            continue;
        if (c == '\n') {
            if (rakhub_usb_line_len == 0)
                continue;
            rakhub_usb_line[rakhub_usb_line_len] = '\0';
            rakhubUsbHandleLine(reinterpret_cast<char *>(rakhub_usb_line));
            rakhub_usb_line_len = 0;
            continue;
        }
        if (rakhub_usb_line_len + 1 >= sizeof(rakhub_usb_line)) {
            rakhub_usb_line_len = 0;
            LOG_INFO("RAKHUB: line overflow, discarded");
            continue;
        }
        rakhub_usb_line[rakhub_usb_line_len++] = (char)c;
    }
}

#endif // RAK_SENSORHUB_DOWNLINK_POC && RAK_SENSORHUB_USB_PROFILE

/**
 * 1-Wire poll task (called periodically): send get.data(pid) when link is idle.
 * Includes hot-plug listen window (3 s no TX every 60 s), discovery get.data(0x01..0x04),
 * and round-robin poll (~1.5 s per PID). Returns suggested next call interval (ms).
 */
static int32_t onewirePollHandle()
{
    const uint32_t now = millis();
    concurrency::LockGuard guard(&onewireLock);

#if RAK_SENSORHUB_DOWNLINK_POC && RAK_SENSORHUB_USB_PROFILE
    rakhubUsbPollSerial();
#endif

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

#if RAK_SENSORHUB_DOWNLINK_POC
    if (link_idle && downlink_poc_pending && (now - last_err_time) > 300) {
        if (sendNextDownlinkPoc())
            return 100;
    }
#endif

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

/** Fill measurement variant from env cache. When which_variant is air_quality_metrics, only CO2 is filled (ppm). Otherwise environment_metrics (temp, humidity, pressure, wind, soil, radiation, power). Returns true if any field was set. */
bool RAKSensorHub::getMetrics(meshtastic_Telemetry *measurement)
{
    bool any = false;
    const uint32_t now = millis();
    // Allow cached readings to be reused for a while because 1-Wire frames can be
    // missed under BLE/app load. Five minutes keeps outdoor use stable.
    const uint32_t maxAgeMs = 5 * 60 * 1000;

    // CO2 is reported in AirQualityMetrics, not EnvironmentMetrics
    if (measurement->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
        if (scalarFresh(env.co2, now, maxAgeMs)) {
            measurement->variant.air_quality_metrics.has_co2 = true;
            measurement->variant.air_quality_metrics.co2 = (uint32_t)(env.co2.value <= 0 ? 0 : (env.co2.value > 5000 ? 5000 : env.co2.value));
            return true;
        }
        return false;
    }

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
    if (scalarFresh(env.distance, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_distance = true;
        measurement->variant.environment_metrics.distance = env.distance.value; // Distance in mm (used for water level detection).
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
    if (scalarFresh(env.moisture, now, maxAgeMs)) {
        measurement->variant.environment_metrics.has_soil_moisture = true;
        measurement->variant.environment_metrics.soil_moisture =
            (uint8_t)(env.moisture.value < 0 ? 0 : (env.moisture.value > 100 ? 100 : (uint32_t)env.moisture.value));
        any = true;
    }
    if (scalarFresh(env.high_precision_humidity, now, maxAgeMs)) {
        // IPSO 0x70 is "high precision humidity" and is used by multiple sensors (soil / weather station).
        // Expose it in both fields so clients can display it without special-casing.
        float v = env.high_precision_humidity.value;

        // Float humidity (%RH)
        if (!measurement->variant.environment_metrics.has_relative_humidity) {
            measurement->variant.environment_metrics.has_relative_humidity = true;
            measurement->variant.environment_metrics.relative_humidity = v;
        }

        // Integer percent field (0..100)
        if (!measurement->variant.environment_metrics.has_soil_moisture) {
            measurement->variant.environment_metrics.has_soil_moisture = true;
            measurement->variant.environment_metrics.soil_moisture = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : (uint32_t)v));
        }
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

#if defined(RAK_SENSORHUB_USB_PROFILE) && RAK_SENSORHUB_USB_PROFILE
void rakhubNotifyProfileFileUploaded(const char *filename)
{
    LOG_INFO("RAKSensorHub: profile uploaded (POC stub): %s", filename ? filename : "(null)");
}
#endif

#endif // HAS_RAKHUB  
