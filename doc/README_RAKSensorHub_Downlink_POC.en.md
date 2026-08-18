# RAKSensorHub Downlink IOC POC (Handoff)

**Design refs**: `doc/RAKSensorHub_Next_Phase_Design.md`, `doc/RAKSensorHub_Architecture_en.md`, `doc/SensorHub_Architecture_v0.1.md`, `doc/SensorHub_Interface_v0.1.md`, `doc/RAKSensorHub上行版本支持的传感器列表.md`, repo root `README(Add_RAKSensorHub).md`.

---

## Downlink POC commits already in Git

| Commit | Summary |
|--------|---------|
| `3e93e8fc8` | RAKSensorHub: downlink IOC POC (`IO_ADDPOLLEX` and related), Hub-side sequences aligned with ProbeIO core-1.2.27. |
| `f12f0819a` | AIC (4–20 mA) `IO_DECODE` downlink and parse path. |
| `bcd835654` | GE JSON downlink: `DOWNLINK_AUTO=0`, `StreamAPI`→`rakhubUsbFeedByte`, `battery_lite` / `rk300_03` JSON, CO2 parse + log throttling. |

(Earlier commits such as `fe0171fb0` are uplink-parse improvements, not the downlink POC core.)

**Shipped in USB POC batch**: USB CDC `RAKHUB …` lines; `RAK_SENSORHUB_USB_PROFILE`; `bin/rakhub_usb_poc.py`; XMODEM stub. **Do not** share the CDC port with the Meshtastic protobuf client.

**2026-08-17 bench (ProbeIO GE, core-1.2.27)**: `battery_lite_core-1.2.27.json` (RAK9154 @ Modbus `0x6E`) and `rk300_03_core-1.2.27.json` (CO₂ @ slave `01`, IPSO 125 → **~985 ppm**). Details: `doc/RAKSensorHub_CHANGELOG.md` §〇-B.

---

## 1. What it is

On **rak2560 + HAS_RAKHUB**, the firmware sends **IOC frames** over **OneWire** to **ProbeIO** (e.g. `IO_RMPOLL`, `IO_RMPDEF`, `IO_CFG`, `IO_ADDPOLLEX`, `IO_ENABLEPOLL`, `IO_PSM`, `IO_DECODE`) to configure RS485 / AIC interfaces and polling tasks so third-party sensors can be provisioned without WisToolBox.

---

## 2. What it can do

- **Build-time**: select a built-in downlink template via `RAK_SENSORHUB_DOWNLINK_TEMPLATE` for SKU-specific multi-task RS485 / AIC POC.
- **Runtime (experimental)**: USB text lines to override RS485/AIC parameters and `APPLY` to re-run clear → Probe soft reboot → re-provision flow.
- **`RAK_SENSORHUB_DOWNLINK_AUTO=0`** (recommended for field trials): Hub reboot / hot-plug **does not** auto-schedule clear-only downlink; ProbeIO EEPROM tasks survive until the next manual `RAKHUB APPLY`.
- **WisToolBox JSON → Hub**: `bin/rakhub_usb_poc.py json --apply --file bin/*.json` (see cheat sheet). ProbeIO must be in **GE** device mode — not core built-in drivers (`dtype=RS` for RAK9154, etc.).

---

## 3. How it works

- **Protocol**: IOC over OneWire; `RAK-OneWireSerial` exposes `RakSNHub_IOC_AddPollEx` etc.; `RAKSensorHub.cpp` queues IOCs and parses responses.
- **Build**: `RAK_SENSORHUB_DOWNLINK_POC=1` in `variants/nrf52840/rak2560/platformio.ini`; template id from macro.
- **Uplink**: IPSO payloads → `EnvCache` → `getMetrics()` → Meshtastic telemetry (see Chinese sensor list doc for IPSO table).
- **Visibility**: Only IPSO types that are **mapped into protobuf** appear in the app / mesh telemetry. Other IPSOs may still show in **USB serial logs** (e.g. `+EVT:… IPSO[…]`); extending firmware + protobuf is required to surface them as telemetry fields.

---

## 4. What is already done

- Hub downlink sequences aligned with core-1.2.27; `IO_ADDPOLLEX` extended-task POC.
- AIC `IO_DECODE` IOC path.
- USB `RAKHUB` + `StreamAPI` byte forwarding (`rakhubUsbFeedByte`) so APPLY works while protobuf client is idle on other bytes.
- **GE RS485 JSON profiles** verified: `bin/battery_lite_core-1.2.27.json`, `bin/rk300_03_core-1.2.27.json`.
- CO₂ uplink: `parseCo2Ipso()` (IPSO 0x7D, little-endian ppm) → `AirQualityMetrics`; logs only on value change.
- Multi-IPSO uplink mapping; poll/RSP noise reduced (`LOG_DEBUG` for routine `+EVT:RSP`).

---

## 5. Next / in progress

| Track | Items |
|-------|--------|
| **Next** | Config persistence and single source of truth; `meshtastic-python-cli` or protobuf admin path; end-to-end matrix for custom RS485 IPSO → telemetry; cross-iface bulk clear; pause routine `get.data` during IOC bursts to cut `SEQUCE/CHKSUM`. |
| **In progress** | USB text POC; `bin/rakhub_usb_poc.py` as the minimal helper for the same USB text path; local `lib_extra_dirs` for OneWire lib iteration; XMODEM profile stub. |

### USB / `bin/rakhub_usb_poc.py` command cheat sheet

**Prereq**: `python -m pip install pyserial`.

**Rule**: disconnect the Meshtastic app / protobuf client from that USB serial while sending lines. Default CDC baud is `115200` (override with `--baudrate`).

| Goal | Example |
|------|---------|
| Hub USB override status | `python bin/rakhub_usb_poc.py --port COM7 status` |
| Single RS485 profile + APPLY | `python bin/rakhub_usb_poc.py --port COM7 --apply rs485 --baud 4800 --hex 010300120001 --ipso 112 --scale 0.1 --name GE` |
| Multi RS485 (`slot=0..3`, one `--task-spec` per line) | `python bin/rakhub_usb_poc.py --port COM7 --apply rs485 --baud 4800 --task-spec "task=1,hex=010300000001,ipso=112,scale=0.1,name=GE" --task-spec "task=2,hex=010300010001,ipso=103,scale=0.1,name=PH"` |
| AIC / 4–20 mA + APPLY | `python bin/rakhub_usb_poc.py --port COM7 --apply aic --ch 1 --ipso 130 --min 0 --max 5 --name ULB16_05` |
| DI (digital input) + APPLY | `python bin/rakhub_usb_poc.py --port COM7 --apply di --ch 1 --trigger 2 --debounce 100 --name GE` |
| DI from WisToolBox JSON | `python bin/rakhub_usb_poc.py --port COM7 --apply json --file bin/DI_01.json --subst PRB_ID=01 --subst PROFILE_NAME=GE` |
| **RAK9154 battery (GE JSON)** | `python bin/rakhub_usb_poc.py --port COM7 --apply json --file bin/battery_lite_core-1.2.27.json --hub-ready-sec 0 --post-apply-lines 400` |
| **RK300-03 CO₂ (GE JSON)** | `python bin/rakhub_usb_poc.py --port COM7 --apply json --file bin/rk300_03_core-1.2.27.json --hub-ready-sec 0 --post-apply-lines 400` |
| Built-in template id (`0` clear, `6` DI, `4` AIC, …) + APPLY | `python bin/rakhub_usb_poc.py --port COM7 --apply builtin 6` |
| APPLY with explicit Probe PID | add `--pid 01` → sends `RAKHUB APPLY 01` |

**JSON / `slot=` notes**

- WisToolBox `json` subcommand always emits **`slot=0`** for the first (and each) RS485 task line so Hub RAM is cleared — avoids stacking stale tasks (e.g. wrong `DEV_ADDR=02` left in slot 1).
- Stop `pio device monitor` (or any tool holding COM) before running the script.
- After `queued all IOC commands`, wait **one poll period** (~60 s) before expecting non-zero IPSO values.
- `+ERR:SEQUCE` during APPLY is common; confirm with `IOC RSP: func=IO_ADDPOLLEX` / `IO_ENABLEPOLL`.
- Wrong Modbus slave (e.g. `02` when sensor is `01`) yields `IPSO[7d]` with payload **`7D 00 00`** until re-APPLY with correct hex.

**GE vs built-in ProbeIO drivers**

- RAK9154 can use core **`dtype=RS`** built-in driver (`do_RAK9154_*` in `app_start.c`) **or** Hub-downlinked **GE** Modbus tasks (`battery_lite` JSON, slave **`0x6E`**). This POC uses the **GE** path only.
- RK300-03 has **no** built-in driver; use GE + IPSO **125** (`0x7D`), default Modbus **`010300000001`** (slave `01`, reg `0`).

**DI notes (core-1.2.27)**: ProbeIO must be **GE** mode. After APPLY, firmware sends **post-IO_DECODE reboot** so `rak_di_init()` applies PB13. Expect **`DI REPORT IPSO[00]`** on **edge** (toggle ProbeIO DI1), not temperature from DS18B20 on Hub `WB_IO1`.

Manual serial is equivalent: send `RAKHUB RS485 …` / `RAKHUB AIC …`, then `RAKHUB APPLY\n`. For multi-task firmware, use explicit **`slot=0`** on the first line to reset USB template RAM.

---

## Minimal file set to commit **with this USB POC**

Follow the documentation map in repo root **`README(Add_RAKSensorHub).md`**. You do **not** need to add every untracked asset under `doc/` (PDFs, slides, large images) in the same commit as the USB POC—split if preferred.

**Firmware / flags**

- `src/modules/Telemetry/Sensor/RAKSensorHub.cpp` (and `RAKSensorHub.h` if touched)
- `src/modules/Telemetry/Sensor/RAKSensorHubProfile.h`
- `src/xmodem.cpp`
- `variants/nrf52840/rak2560/platformio.ini` (**sanitize `lib_extra_dirs`** before push—no personal `D:\...` paths)

**Helper**

- `bin/rakhub_usb_poc.py`
- `bin/battery_lite_core-1.2.27.json` — RAK9154 GE RS485 (4 Modbus tasks)
- `bin/rk300_03_core-1.2.27.json` — RK300-03 CO₂ GE RS485
- `bin/DI_01.json` (WisToolBox-style DI template sample)
- `bin/wistool_SDSIN_SOIL_4IN1.json` (RS485 sample)

**Dependency**: local `RAK-OneWireSerial` must include `RakSNHub_IOC_DecodeDI`, `RakSNHub_IOC_ConfigUart`, etc. (commit/tag that lib separately or use `lib_extra_dirs` until published).

**Docs (keep consistent with behavior)**

- `README(Add_RAKSensorHub).md`
- `doc/README_RAKSensorHub_Downlink_POC.zh.md`, `doc/README_RAKSensorHub_Downlink_POC.en.md`
- `doc/RAKSensorHub_CHANGELOG.md` (replace the placeholder hash in section 1 after you commit)
- `doc/RAKSensorHub_Architecture_zh.md`, `doc/RAKSensorHub_Architecture_en.md`
- `doc/RAKSensorHub上行版本支持的传感器列表.md`, `doc/RAKSensorHub_Uplink_Sensor_Support_List.en.md`
- `doc/RAKSensorHub&Meshtastic Python CLI 方案分析.md` (if part of this delivery)

**Avoid mixing into the same commit**

- `src/main.cpp`, `src/modules/Telemetry/EnvironmentTelemetry.cpp` (current diffs are **unrelated** to SensorHub USB—revert or commit separately with rationale).

**Do not commit**

- `.agents/`, personal `userPrefs.jsonc`, scratch binaries, unless the team explicitly wants them.
