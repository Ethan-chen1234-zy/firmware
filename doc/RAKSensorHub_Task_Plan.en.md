# RAKSensorHub Task Plan

> **Audience**: New engineers taking over RAK2560 / WisMesh Hub + ProbeIO firmware and tooling.  
> **Scope**: Two tracks — **data plane** (ProbeIO uplink IPSO → Meshtastic telemetry) and **config plane** (downlink IOC / USB CLI / future Admin).  
> **Code**: `src/modules/Telemetry/Sensor/RAKSensorHub.{h,cpp}`, `bin/rakhub_usb_poc.py`, `bin/regen-protos.sh`.  
> **ProbeIO reference**: `core-1.2.27` (aligned with shipped ProbeIO).  
> **Chinese version**: [`RAKSensorHub_Task_Plan.md`](RAKSensorHub_Task_Plan.md)

**Document version**: 0.9 · **Last reviewed**: 2026-05-22

---

## 0. For new teammates: status, direction, is this plan sound?

### 0.1 Soundness (verdict: good structure for onboarding)

| Aspect | Verdict | Notes |
|--------|---------|-------|
| Dual-track split (uplink / downlink) | ✅ Sound | Matches `RAKSensorHub.cpp` and ProbeIO IOC model |
| Sprint priorities | ✅ Sound | Close **EnvCache → proto → getMetrics** first, then **IOC timing (D2-5)** |
| Dependencies | ✅ Sound | D1-4 and D1-5 **in parallel**; mapping doc must not block proto merge |
| Task IDs | ✅ Clear | **D1-\*** = data plane (uplink), **D2-\*** = config plane (downlink) |
| Pitfalls | ⚠️ Must read | §0.4; DI / DS18B20 / post-IO_DECODE reboot are the top misreads |

**Scope note**: This is a **RAK2560 + ProbeIO POC** roadmap, not a full upstream Meshtastic commitment. Merging `telemetry.proto` needs separate coordination.

### 0.2 Current status (2026-05-22, monolith @ v2.7.25)

**One-liner**: Sprint 1 **P0 done in tree** — `telemetry.proto` fields 24–26, `getMetrics()` exports EC/pH/salinity, D2-5 IOC RSP timeout tuned, `rakhub_usb_poc.py` hub-ready wait. **Hardware re-verify** on bench still required.

| Capability | Status | Evidence |
|------------|--------|----------|
| Downlink RS485 / SDI-12 / RS232 / AIC / AIV / DI / DO templates | ✅ Demo | `BUILTIN 0–9`; `RAKHUB DI`; `bin/DI_01.json` + `rakhub_usb_poc.py` |
| Downlink USB JSON import | ✅ RS485 / AIC / DI | DI: `Interface: AI` + `io_decode=…:di:…` |
| Post-IO_DECODE reboot (DI/DO) | ✅ | Firmware reboots after `IO_DECODE` so `rak_di_init()` applies PB13 |
| Pause `get.data` during downlink | ✅ Partial | While `downlink_poc_pending` / `wait_rejoin` |
| Uplink IPSO parse → EnvCache | ✅ Partial | Temp/humidity/wind/soil/EC/pH/salinity + **DI/DO** |
| Uplink → App (`getMetrics`) | ✅ P0 | `soil_conductivity`, `soil_ph`, `salinity` (fields 24–26) |
| `telemetry.proto` new fields | ✅ P0 | `protobufs/` submodule + patched `telemetry.pb.h` (regen script needs nanopb bundle on host) |
| Uplink IPSO=0 (DI) logging | 🔄 Partial | `SDATA`/`REPORT` + `EnvCache.digital_input`; edge-triggered |
| IOC production stability | 🔄 Improved | Poll paused; 5 s IOC RSP timeout (no get.data PID rotate); gap 400 ms — bench retest |
| Admin / NVS remote config | ❌ Not started | Sprint 3 |

### 0.3 Direction (next 1–2 iterations)

```text
┌─ Sprint 1 (must-do) ───────────────────────────────────────────┐
│ Data: D1-4 proto 24–26 ∥ D1-5 mapping → D1-1 getMetrics export │
│ Config: D2-5 finish (per-step RSP wait) + GE mode checklist     │
└─────────────────────────────────────────────────────────────────┘
         ↓
┌─ Sprint 2 (expand) ────────────────────────────────────────────┐
│ D1-2/3 more IPSOs; D1-7 DI uplink; D2-4 water/NPK preset JSON  │
└─────────────────────────────────────────────────────────────────┘
         ↓
┌─ Sprint 3 (productize) ────────────────────────────────────────┐
│ D2-6 NVS persist; D2-7 Admin protobuf; D1-8 parser refactor     │
└─────────────────────────────────────────────────────────────────┘
```

### 0.4 Pitfalls (troubleshooting)

| Symptom | Common wrong assumption | Actual cause |
|---------|-------------------------|--------------|
| `get.data` only shows `IPSO[82]` | DI downlink failed | Usually **stale AIC cache**; DI is **edge-driven**, not always in poll responses |
| DS18B20 temperature on RAK4631 | DI should show temperature | **Hub `WB_IO1` (Dallas 1-Wire) ≠ ProbeIO DI (PB13)**; DI reports **0/1** only |
| Sent `IO_DECODE(DI)` but no DI data | Frames ignored | ProbeIO needs **GE mode** + **REBOOT** so `rak_di_init()` applies GPIO |
| `+ERR:SEQUCE` | Broken ProbeIO | Hub still polls `get.data` during IOC; need **D2-5** |
| EC in EnvCache but not in App | Parse bug | **D1-1** missing: `getMetrics()` not mapped to protobuf |

---

## 1. Task summary

**Legend**: ✅ Done · 🔄 Partial / in progress · ❌ Not started · ⚠️ Recommended / non-blocking gap

### 1.1 Config plane (Downlink) D2-*

| ID | Task | Status | Priority | Notes |
|----|------|--------|----------|-------|
| D2-1 | SDI-12 USB / built-in template | ✅ | P1 | `IOC_SDI12`, `BUILTIN 5` |
| D2-2 | DI / DO USB CLI + JSON | ✅ DI · ⚠️ DO | P2 | `RAKHUB DI`; DO has `BUILTIN 8`, USB `do` subcommand TBD |
| D2-3 | RS232 downlink | ✅ | P2 | `BUILTIN 7` |
| D2-4 | Preset templates (NPK, water quality, …) | 🔄 | P2 | `BUILTIN 0–9` exist; need business JSON library |
| D2-5 | IOC throttle, RSP gate, REBOOT after DI | 🔄 | **P0** | Pause poll + IOC_RSP clears waiter; 5 s timeout; script `--hub-ready-sec` |
| D2-6 | USB template NVS persistence | ❌ | P3 | Survive power cycle |
| D2-7 | `SensorHubConfig` Admin protobuf | ❌ | P2 | Depends on D1-4, D2-6 |

### 1.2 Data plane (Uplink) D1-*

| ID | Task | Status | Priority | Notes |
|----|------|--------|----------|-------|
| D1-4 | `telemetry.proto` fields 24–26 + regen | ✅ | **P0** | `soil_conductivity`, `soil_ph`, `salinity` |
| D1-5 | IPSO ↔ EnvCache ↔ Proto mapping doc | ✅ | **P0** | `RAKSensorHub_Uplink_Sensor_Support_List.en.md` §4 |
| D1-1 | `getMetrics()` export ec / ph / salinity | ✅ | **P0** | `RAKSensorHub.cpp` |
| D1-6 | `getMetrics()` export new IPSO fields | ❌ | P1 | After D1-3, D1-4 |
| D1-2 | EnvCache extension (NPK, water, PM, …) | ❌ | P2 | New `env.*` scalars |
| D1-3 | `onewire_evt()` new IPSO branches | ❌ | P2 | After D1-2 |
| D1-7 | Uplink IPSO=0 (DI) parse + log | 🔄 | P2 | Parsed + logged; Mesh/App export TBD |
| D1-8 | Refactor duplicate IPSO switches | ❌ | P3 | Tech debt |

### 1.3 D1-* item-by-item (data plane / uplink)

| ID | Short name | What to do | Status | Done when |
|----|------------|------------|--------|-----------|
| **D1-1** | getMetrics soil trio | Export existing `ec`, `soil_ph`, `salinity` from EnvCache to `EnvironmentMetrics` / App | ✅ | App shows EC (mS/cm), pH, salinity (mg/L), not `distance` |
| **D1-2** | EnvCache extension | Add cache fields for NPK, water quality, PM2.5, etc. | ❌ | New scalars in `RAKSensorHub.h` |
| **D1-3** | onewire_evt IPSO | New `switch` branches → `setScalar` for new IPSOs | ❌ | Serial LOG + EnvCache values |
| **D1-4** | telemetry.proto | Add EnvironmentMetrics fields 24+; run `regen-protos.sh` | ✅ | `.pb.h` updated; build passes |
| **D1-5** | Mapping table doc | Maintain IPSO ↔ quantity ↔ EnvCache ↔ proto field table | ✅ | §4 in uplink list |
| **D1-6** | getMetrics bulk export | Export all D1-2/3 fields in `getMetrics()` | ❌ | Needs D1-4 proto fields |
| **D1-7** | DI uplink | Parse **IPSO=0**, 1-byte 0/1, edge-triggered reports | 🔄 | `DI REPORT IPSO[00]` in log; Mesh/App TBD |
| **D1-8** | Parser refactor | Unify duplicate IPSO switches into `parseIpso()` | ❌ | Same behavior, cleaner code |

**Suggested order (Sprint 1)**: D1-5 ∥ D1-4 → **D1-1** → (Sprint 2) D1-2 → D1-3 → D1-6, D1-7.

### 1.4 D2-* item-by-item (config plane / downlink)

| ID | Short name | What to do | Status | Done when |
|----|------------|------------|--------|-----------|
| **D2-1** | SDI-12 downlink | `IO_CFG` + `IO_ADDPOLLEX` for SDI-12 sensors | ✅ | `BUILTIN 5` or JSON; ProbeIO polls on schedule |
| **D2-2** | DI / DO | `IO_DECODE` for digital I/O; USB `RAKHUB DI` + `DI_01.json` | ✅ DI / ⚠️ DO | DI: `IO_PSM`+`IO_DECODE` in log; DO: `BUILTIN 8`, no `rakhub do` yet |
| **D2-3** | RS232 downlink | Serial passthrough poll template | ✅ | `BUILTIN 7` |
| **D2-4** | Template library | Water / NPK WisToolBox JSON → `BUILTIN` or docs | 🔄 | Reproducible JSON + command per sensor class |
| **D2-5** | IOC hardening | Pause `get.data` during IOC; wait for RSP; **REBOOT** after DI | 🔄 | Done: pause poll + post-decode reboot; TBD: stepwise IOC |
| **D2-6** | NVS persistence | Store USB template in Flash across Hub reboot | ❌ | `RAKHUB STATUS` still shows template after power cycle |
| **D2-7** | Admin remote config | Protobuf SensorHub config (replace USB-only POC) | ❌ | App/CLI can APPLY equivalent config |

**Suggested order (Sprint 1)**: **D2-5** in parallel with D1-1.

### 1.5 Cross-cutting (no task ID)

| Item | Status | Notes |
|------|--------|-------|
| ProbeIO `dtype=GE` prerequisite | ⚠️ Documented | RC mode has no `rak_di_init` stack |
| `rakhub_usb_poc.py` builtin range 0–9 | ✅ | — |
| Meshtastic App vs `RAKHUB` on USB CDC | ⚠️ Ops | Disconnect App protobuf during downlink POC |

---

## 2. Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│  Config plane (Downlink)                                         │
│  USB text RAKHUB … / JSON templates / future Admin protobuf      │
│       → 1-Wire IOC (IO_CFG / IO_DECODE / IO_ADDPOLLEX …)         │
│       → ProbeIO EEPROM + drivers (GE mode, etc.)                 │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│  Data plane (Uplink)                                             │
│  ProbeIO IPSO payloads → onewire_evt() → EnvCache                │
│       → getMetrics() → EnvironmentMetrics / AirQualityMetrics    │
│       → App / MQTT / Mesh telemetry                              │
└─────────────────────────────────────────────────────────────────┘
```

**WisToolBox JSON**: `ATC+…` lines describe ProbeIO-local semantics. Meshtastic converts them to **1-Wire IOC frames** via `RAKHUB` / built-in templates — AT strings are not sent verbatim to ProbeIO.

---

## 3. Workspace and Protobuf

| Resource | Path | Notes |
|----------|------|-------|
| Firmware repo | `Meshtastic/firmware` | This plan lives here |
| Protobuf | Submodule `protobufs/` | Regen after `.proto` edits |
| Generated code | `src/mesh/generated/meshtastic/*.pb.h` | **Do not edit by hand** |

```bash
# From firmware root
./bin/regen-protos.sh
```

**Policy**: Add **fields 24+** on a branch and regen locally; coordinate before merging upstream. Check existing EnvironmentMetrics fields 1–23 first.

---

## 4. Data plane details (D1-*)

### 4.1 D1-1 — parsed but not exported (Sprint 1 top priority)

| EnvCache | IPSO | Proposed proto | Field # |
|----------|------|----------------|---------|
| `ec` | 0xC0 | `float ec` (mS/cm) | 24 |
| `soil_ph` | 0xC1 / 0xC2 | `float ph` | 25 |
| `salinity` | 0x13 | `float salinity` (mg/L) | 26 |

**Acceptance**: App shows EC, pH, salinity on **named fields**, not repurposed `distance`.

### 4.2 D1-7 — digital input uplink

- DI reports **IPSO = 0**, payload **1 byte 0/1**, **edge-triggered**.
- **Cannot** read DS18B20 temperature via DI (Hub `WB_IO1` ≠ ProbeIO **PB13**).

### 4.3 More IPSOs (Sprint 2)

See `RAKSensorHub_Uplink_Sensor_Support_List.en.md`: NPK 0x10–0x12, water 0x14–0x1A, PM/VOC, etc.

---

## 5. Config plane details (D2-*)

### 5.1 IOC flow by interface

| Interface | Flow | Type |
|-----------|------|------|
| RS485 / RS232 / SDI-12 | Clear → reboot → `IO_CFG` → `IO_ADDPOLLEX` → `IO_ENABLEPOLL` | Polled |
| AIC / AIV | Optional `IO_PSM` → `IO_DECODE` | Decode page |
| DI | Optional `IO_PSM` → `IO_DECODE` (no IO_CFG) | **Event**; recommend **REBOOT** at end |
| DO | `IO_DECODE` | Control |

**BUILTIN reference**:

| ID | Template |
|----|----------|
| 0 | Clear only |
| 1 | JXBS3001-EC |
| 2 | SDSIN soil 4-in-1 |
| 3 | JXBS4001-pH |
| 4 | AIC 4–20mA |
| 5 | SDI-12 |
| 6 | DI |
| 7 | RS232 |
| 8 | DO |
| 9 | AIV |

**Import DI from JSON**:

```bash
python bin/rakhub_usb_poc.py --port COMx --apply json --file bin/DI_01.json \
  --subst PRB_ID=01 --subst PROFILE_NAME=GE --subst SNSR_INTV=60
```

Or: `python bin/rakhub_usb_poc.py --port COMx --apply builtin 6` (disconnect Meshtastic App from USB).

### 5.2 D2-5 — production hardening

| Problem | Target |
|---------|--------|
| `get.data` during IOC | Pause polling until IOC sequence completes |
| Fixed step timing | Advance only after matching RSP; timeout/retry |
| DI only writes EEPROM | Send **ProbeIO REBOOT** at end of config phase |

---

## 6. Suggested iteration order

```text
Sprint 1: D1-5 ∥ D1-4 → D1-1 → D2-5
Sprint 2: D1-2, D1-3, D1-6, D1-7, D2-4
Sprint 3: D2-6, D2-7, D1-8
```

---

## 7. Onboarding checklist

| Document | Language |
|----------|----------|
| This file / `RAKSensorHub_Task_Plan.md` | EN / 中文 |
| `RAKSensorHub_Architecture_zh.md` / `_en.md` | Architecture |
| `RAKSensorHub_Code_Map_for_New_Dev.md` | Code map |
| `RAKSensorHub_Uplink_Sensor_Support_List.en.md` | Uplink IPSO |

1. Read `.github/copilot-instructions.md` (MCP / hardware tests if you have devices).
2. `pio run -e rak2560`; confirm `HAS_RAKHUB=1`, `RAK_SENSORHUB_DOWNLINK_POC=1`.
3. Logs: `pio device monitor` or `python bin/rakhub_usb_poc.py --port COMx monitor`.
4. Downlink POC: **disconnect** App USB protobuf.
5. After proto edits: `./bin/regen-protos.sh` and full rebuild.

---

## 8. Revision history

| Version | Date | Notes |
|---------|------|-------|
| 0.9 | 2026-05-22 | **P0 Sprint 1** on monolith v2.7.25: D1-4/5/1, D2-5 timeout, `rakhub_usb_poc.py --hub-ready-sec`; see `RAKSensorHub_POC_Sprint1_Verification.md` |
| 0.8 | 2026-05-19 | v2.8.0 split tag local archive; branch reset to v2.7.25 monolith |
| 0.7 | 2026-05-19 | Multi-file split phase 1–3; tags v2.7.25 / v2.8.0 |
| 0.6 | 2026-05-15 | DI post-reboot, IPSO 00/01 uplink, partial D2-5; USB/JSON commit notes |
| 0.5 | 2026-05-15 | §1.3/§1.4 per-task tables; BUILTIN table; English file created |
| 0.4 | 2026-05 | §0 status/direction; §1 summary tables |
| 0.3 | 2026-05 | §0 assessment |
| 0.2 | 2026-05 | DI semantics |
| 0.1 | — | Initial draft |

**Task IDs**: data plane **D1-\***, config plane **D2-\*** (use same IDs in PRs / issues).
