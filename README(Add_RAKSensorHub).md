# RAK SensorHub — Project entry (`README(Add_RAKSensorHub).md`)

**Purpose:** This file is the **single entry** for developers working on RAK SensorHub / Probe IO inside Meshtastic on branch `feature/add_RAKSensorHub`. Use it to find **primary sources**, **what to edit**, **build flags**, and **how capabilities map to versions**. Detailed commit hashes live in `doc/RAKSensorHub_CHANGELOG.md` (we **do not** duplicate that table here—update the changelog file when you land commits).

**Chinese design / PRD:** see `doc/RAKSensorHub_Architecture_zh.md`, `doc/README_RAKSensorHub_Downlink_POC.zh.md`, `doc/SensorHub_PRD_v0.1.md`.

---

## 1. Documentation map

| Topic | Path |
|-------|------|
| Architecture (EN / ZH) | `doc/RAKSensorHub_Architecture_en.md`, `doc/RAKSensorHub_Architecture_zh.md` |
| Downlink IOC POC (EN / ZH) | `doc/README_RAKSensorHub_Downlink_POC.en.md`, `doc/README_RAKSensorHub_Downlink_POC.zh.md` |
| Roadmap / next phase | `doc/RAKSensorHub_Next_Phase_Design.md` (+ `.en.md`) |
| Uplink IPSO ↔ sensors | `doc/RAKSensorHub_Uplink_Sensor_Support_List.en.md` |
| CLI vs USB POC | `doc/RAKSensorHub&Meshtastic Python CLI 方案分析.md` |
| **Commit-level changelog** | **`doc/RAKSensorHub_CHANGELOG.md`** ← append rows here after each meaningful merge |

---

## 2. Changelog: README vs `doc/RAKSensorHub_CHANGELOG.md`

- **Keep** `doc/RAKSensorHub_CHANGELOG.md` as the **authoritative git-derived history** (hash, date, one-line subject).
- **This README** carries only **milestone semantics** so newcomers know “what generation of code” they are on:

| Milestone | Meaning (branch narrative) |
|-----------|-----------------------------|
| M0 | SensorHub / `RAKSensorHub` introduced (`b77d9c94d`, `7efae8b61`). |
| M1 | Uplink parse improvements (`fe0171fb0`). |
| M2 | **Downlink IOC POC** on Hub (`IO_ADDPOLLEX` etc., `3e93e8fc8`). |
| M3 | **AIC** `IO_DECODE` path (`f12f0819a`). |
| M4 | Merge `develop`, docs / `platformio` hygiene (`e94e21094`, `9cf699d45`, `ecac05a5e`). |
| M5 | **USB CDC `RAKHUB` + `bin/rakhub_usb_poc.py`** (`RAK_SENSORHUB_USB_PROFILE`), multi-task `slot=`, XMODEM hook stub; changelog row = `doc/RAKSensorHub_CHANGELOG.md` §一 首行（提交后补短 hash）。 |

For the full table, open **`doc/RAKSensorHub_CHANGELOG.md`**.

---

## 3. Primary files (read these first)

| Area | Path | Role |
|------|------|------|
| Hub driver | `src/modules/Telemetry/Sensor/RAKSensorHub.cpp`, `.h` | OneWire poll/RX, IPSO → `EnvCache`, downlink POC state machine, optional USB `RAKHUB` parser |
| Telemetry export | `src/modules/Telemetry/EnvironmentTelemetry.cpp`, `AirQualityTelemetry.cpp` | Maps hub metrics into Meshtastic protobuf for mesh/app |
| Board flags | `variants/nrf52840/rak2560/platformio.ini` | **`HAS_RAKHUB`**, **`RAK_SENSORHUB_DOWNLINK_POC`**, **`RAK_SENSORHUB_DOWNLINK_TEMPLATE`**, **`RAK_SENSORHUB_USB_PROFILE`**, `lib_extra_dirs` → `RAK-OneWireSerial` |
| OneWire library | **`RAK-OneWireSerial`** (see `lib_extra_dirs` in `platformio.ini`) | IOC helpers, framing, `RakSNHub_IOC_*` |
| USB POC helper | `bin/rakhub_usb_poc.py` | Sends `RAKHUB …` lines over CDC (pyserial); not Meshtastic protobuf CLI |
| XMODEM hook (stub) | `src/xmodem.cpp`, `src/modules/Telemetry/Sensor/RAKSensorHubProfile.h` | Profile upload callback wiring (POC / future profile file) |

---

## 4. What to change for which task

| Goal | Typical edits |
|------|----------------|
| New IPSO → **App telemetry** | `RAKSensorHub.cpp` (`EnvCache` / parse path / `getMetrics()`), then protobuf + `EnvironmentTelemetry.cpp` (or `AirQualityTelemetry.cpp`) as needed |
| IPSO visible **only in USB log** today | Often **no** telemetry change yet—confirm in `onewireRxHandle` logs; then same row as above to promote to protobuf |
| **Compile-time** downlink SKU template | **`variants/nrf52840/rak2560/platformio.ini`**: set `-DRAK_SENSORHUB_DOWNLINK_TEMPLATE=N` (see §5). Optionally adjust template tables in `RAKSensorHub.cpp` |
| Enable / disable **downlink IOC engine** | **`platformio.ini`**: `-DRAK_SENSORHUB_DOWNLINK_POC=1` (on) or `0` (off). When off, IOC POC paths are not built |
| **USB `RAKHUB`** runtime profiles | **`platformio.ini`**: `-DRAK_SENSORHUB_USB_PROFILE=1` **and** `RAK_SENSORHUB_DOWNLINK_POC=1`. Logic in `RAKSensorHub.cpp` (`#if RAK_SENSORHUB_DOWNLINK_POC && RAK_SENSORHUB_USB_PROFILE`) |
| OneWire wire format / IOC API | **`RAK-OneWireSerial`** repo, then bump / path in `lib_extra_dirs` |
| Script UX (lab only) | `bin/rakhub_usb_poc.py` |

---

## 5. Build flags (`rak2560`) — how to use each version / mode

Edit **`variants/nrf52840/rak2560/platformio.ini`** under `[env:rak2560] build_flags`:

| Flag | Typical value | Effect |
|------|----------------|--------|
| `HAS_RAKHUB` | `1` | Builds RAK SensorHub path (uses OneWireSerial). **Mutually exclusive** with `HAS_RAKPROT` (see comment in `platformio.ini`). |
| `RAK_SENSORHUB_DOWNLINK_POC` | `1` | Enables Hub → ProbeIO **IOC downlink POC** state machine (`IO_CFG`, `IO_ADDPOLLEX`, …). Set `0` for uplink-only experiments. |
| `RAK_SENSORHUB_DOWNLINK_TEMPLATE` | `0`–`4` | Selects **built-in** `DownlinkSensorTemplate` when USB override is **not** active. Values (must match `RAKSensorHub.cpp`): **`0`** = clear-only (no `IO_CFG` / `IO_ADDPOLLEX` after clear); **`1`** = JXBS-3001-EC RS485 template; **`2`** = SDSIN soil 4-in-1; **`3`** = JXBS-4001-pH; **`4`** = AIC 4–20 mA (`IO_DECODE`). |
| `RAK_SENSORHUB_USB_PROFILE` | `0` / `1` | `1` = USB CDC **`RAKHUB …`** text lines + `rakhubNotifyProfileFileUploaded` stub; requires **`RAK_SENSORHUB_DOWNLINK_POC=1`**. **Disconnect** Meshtastic protobuf clients from that CDC port while typing commands. |

**USB vs compile template:** After flashing, `RAKHUB COMPILE` restores compile-time `RAK_SENSORHUB_DOWNLINK_TEMPLATE`; `RAKHUB RS485` / `RAKHUB AIC` / `RAKHUB BUILTIN` store overrides until `RAKHUB APPLY`. See `doc/README_RAKSensorHub_Downlink_POC.*.md`.

**Probe IO:** After template changes, a **power-cycle / rejoin** is often required—treat as normal until productized.

---

## 6. Current status (snapshot)

**Uplink**

- OneWire to Probe IO is functional for supported builds.
- **Meshtastic telemetry** only shows IPSOs mapped through `getMetrics()` → protobuf. Other IPSOs may appear **only in USB logs**—see `doc/RAKSensorHub_Architecture_en.md` §2.4.

**Downlink / configuration (POC)**

- IOC sequences aligned with Probe IO **core-1.2.27**.
- **Not** WisToolBox inside the Meshtastic app; lab path is **USB `RAKHUB`** or compile-time template + `APPLY` flow.

---

## 7. Target state (short)

- Single configuration schema (protobuf / JSON) shared by firmware, tools, and eventually App.
- Remote admin over Meshtastic transports with explicit security.
- See `doc/RAKSensorHub_Next_Phase_Design.md`.

---

## 8. Hardware reference

| Role | Example |
|------|---------|
| Hub | RAK2560 (`pio run -e rak2560`) |
| IO | Probe IO (RS485 / SDI-12 / 4–20 mA per wiring) |
| Probes / externals | RAK1901/02/04, RK300-03, RK900-09, RK200-03, soil RS485, etc. |

---

## 9. Getting started

```bash
pio run -e rak2560 -t upload --upload-port COMx
```

1. Power / 12 V per RAK hardware docs.  
2. **App telemetry:** connect Meshtastic app; confirm `EnvironmentMetrics` / `AirQualityMetrics`.  
3. **Unmapped IPSO:** watch **USB serial** for `RAKSensorHub` / `+EVT` lines.  
4. **USB POC:** disconnect protobuf on that port; `python bin/rakhub_usb_poc.py --port <COMx> status` or send `RAKHUB HELP`.

### 9.1 USB & `bin/rakhub_usb_poc.py` (quick reference)

**Prereq:** `python -m pip install pyserial`. **Do not** use the same CDC port as the Meshtastic protobuf client while sending commands.

| Action | Command |
|--------|---------|
| Status | `python bin/rakhub_usb_poc.py --port COMx status` |
| RS485 + APPLY | `python bin/rakhub_usb_poc.py --port COMx --apply rs485 --baud 4800 --hex 010300120001 --ipso 112 --scale 0.1 --name GE` |
| Multi RS485 (`--task-spec` → `slot=0,1,…`) | `python bin/rakhub_usb_poc.py --port COMx --apply rs485 --baud 4800 --task-spec "task=1,hex=010300000001,ipso=112,name=Hum" --task-spec "task=2,hex=010300010001,ipso=103,name=PH"` |
| AIC + APPLY | `python bin/rakhub_usb_poc.py --port COMx --apply aic --ch 1 --ipso 130 --min 0 --max 5` |
| Built-in template + APPLY (e.g. clear AIC path) | `python bin/rakhub_usb_poc.py --port COMx --apply builtin 4` |
| APPLY for PID `0x01` | add `--pid 01` to any `--apply` command above |

Full narrative + Chinese copy: `doc/README_RAKSensorHub_Downlink_POC.en.md` / `.zh.md`.

---

## 10. How to help

- Issues: hardware list, **git hash**, `platformio.ini` flag snapshot, USB log excerpt.  
- After merges: add a row to **`doc/RAKSensorHub_CHANGELOG.md`** (or replace the placeholder hash in §1 after your commit); bump §2 milestones here only if the narrative phase changes.

---

## Branch note

Uplink is the most mature path; **downlink + USB `RAKHUB`** remain **POC** until PRD / design milestones for protobuf + remote admin are met (`doc/SensorHub_PRD_v0.1.md`, `doc/RAKSensorHub_Next_Phase_Design.md`).
