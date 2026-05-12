# RAKSensorHub Downlink IOC POC (Handoff)

**Design refs**: `doc/RAKSensorHub_Next_Phase_Design.md`, `doc/RAKSensorHub_Architecture_en.md`, `doc/SensorHub_Architecture_v0.1.md`, `doc/SensorHub_Interface_v0.1.md`, `doc/RAKSensorHub上行版本支持的传感器列表.md`, repo root `README(Add_RAKSensorHub).md`.

---

## Downlink POC commits already in Git

| Commit | Summary |
|--------|---------|
| `3e93e8fc8` | RAKSensorHub: downlink IOC POC (`IO_ADDPOLLEX` and related), Hub-side sequences aligned with ProbeIO core-1.2.27. |
| `f12f0819a` | AIC (4–20 mA) `IO_DECODE` downlink and parse path. |

(Earlier commits such as `fe0171fb0` are uplink-parse improvements, not the downlink POC core.)

**Not described as shipped (direction)**: USB CDC text `RAKHUB …` lines to edit templates at runtime and trigger downlink; optional XMODEM hook stub. Requires exclusive serial when Meshtastic protobuf is active; product path needs dedicated channel or protobuf extension.

---

## 1. What it is

On **rak2560 + HAS_RAKHUB**, the firmware sends **IOC frames** over **OneWire** to **ProbeIO** (e.g. `IO_RMPOLL`, `IO_RMPDEF`, `IO_CFG`, `IO_ADDPOLLEX`, `IO_ENABLEPOLL`, `IO_PSM`, `IO_DECODE`) to configure RS485 / AIC interfaces and polling tasks so third-party sensors can be provisioned without WisToolBox.

---

## 2. What it can do

- **Build-time**: select a built-in downlink template via `RAK_SENSORHUB_DOWNLINK_TEMPLATE` for SKU-specific multi-task RS485 / AIC POC.
- **Runtime (experimental)**: USB text lines to override RS485/AIC parameters and `APPLY` to re-run clear → (recommended) Probe power-cycle → re-provision flow (do not share the CDC port with the Meshtastic protobuf client).

---

## 3. How it works

- **Protocol**: IOC over OneWire; `RAK-OneWireSerial` exposes `RakSNHub_IOC_AddPollEx` etc.; `RAKSensorHub.cpp` queues IOCs and parses responses.
- **Build**: `RAK_SENSORHUB_DOWNLINK_POC=1` in `variants/nrf52840/rak2560/platformio.ini`; template id from macro.
- **Uplink**: IPSO payloads → `EnvCache` → `getMetrics()` → Meshtastic telemetry (see Chinese sensor list doc for IPSO table).

---

## 4. What is already done

- Hub downlink sequences aligned with core-1.2.27; `IO_ADDPOLLEX` extended-task POC.
- AIC `IO_DECODE` IOC path.
- Multi-IPSO uplink mapping and logging (noisy IPSOs still need filtering policy).
- Planning docs: `RAKSensorHub_Next_Phase_Design.md`, etc.

---

## 5. Next / in progress

| Track | Items |
|-------|--------|
| **Next** | Config persistence and single source of truth; `meshtastic-python-cli` or protobuf admin path; end-to-end matrix for custom RS485 IPSO → telemetry; cross-iface bulk clear; pause routine `get.data` during IOC bursts to cut `SEQUCE/CHKSUM`. |
| **In progress** | USB text POC; local `lib_extra_dirs` for OneWire lib iteration; XMODEM profile stub. |

---

## Suggested files to commit (current tree)

- **All of `doc/`** (this README plus existing design/list markdown).
- **`src/modules/Telemetry/Sensor/RAKSensorHub.cpp`** (downlink POC + any USB POC edits).
- **`variants/nrf52840/rak2560/platformio.ini`** (flags; **replace or remove machine-specific `lib_extra_dirs` absolute path** before push).
- **`src/xmodem.cpp`** (`RAK_SENSORHUB_USB_PROFILE` guards).
- **`src/modules/Telemetry/Sensor/RAKSensorHubProfile.h`** (new; XMODEM completion hook).

**Do not commit**: personal `lib_extra_dirs` paths, local `userPrefs.jsonc`, scratch binaries.
