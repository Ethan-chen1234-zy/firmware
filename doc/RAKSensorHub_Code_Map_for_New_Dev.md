## RAKSensorHub: code map for new developers (Uplink → Downlink)

Audience: first-time contributors who previously used the **uplink POC**, and now need a fast pointer to the **downlink IOC POC** changes, code entry points, and the two main add-ons: **IOC verification** and **USB / Python CLI**.

---

### 1) Where to start reading

- **Start from the uplink version**: read `doc/RAKSensorHub_Architecture_en.md` first (it explains the uplink data model and parse path), then check out the uplink tag as a stable baseline.
- **Uplink tag (recommended baseline)**: `v2.7.21-raksensorhub-poc-uplink`
- **Downlink tags (milestones)**:
  - `v2.7.22-raksensorhub-poc-downlink`
  - `v2.7.24-raksensorhub-poc-downlink-v2`

---

### 2) What downlink adds (vs uplink)

Downlink adds two main capabilities:

- **IOC downlink provisioning (Downlink IOC POC)**
  - The Hub (Meshtastic firmware) provisions ProbeIO over OneWire/IOC with: `IO_CFG`, `IO_ADDPOLLEX`, `IO_ENABLEPOLL`, `IO_PSM`, `IO_DECODE`, `IO_RMPOLL`, `IO_RMPDEF`, etc.
  - Goal: configure RS485 polling tasks and AIC (4–20 mA) decode rules without WisToolBox.

- **USB / Python CLI (lab/provisioning entry)**
  - Uses USB CDC text commands `RAKHUB ...` to override templates at runtime and trigger `APPLY` downlink to ProbeIO.
  - Minimal sender tool: `bin/rakhub_usb_poc.py`

Handoff doc: `doc/README_RAKSensorHub_Downlink_POC.en.md`.

---

### 3) Where the code changes live

#### 3.1 Firmware (core logic)

- **`src/modules/Telemetry/Sensor/RAKSensorHub.cpp`**
  - **uplink**: IPSO → EnvCache/HubPower → Telemetry modules
  - **downlink**: IOC downlink state machine, USB `RAKHUB` text command parsing, IOC RSP logs for verification

Downlink entry points to search for (keywords):

- `sendNextDownlinkPoc()` (clear phase / config phase)
- `scheduleDownlinkPoc()` + **`RAK_SENSORHUB_DOWNLINK_AUTO`** (skip auto clear on join when `AUTO=0`)
- `downlink_poc_*` (state + pacing, e.g. `downlink_poc_step/phase/pending/next_ms`)
- `IO_ADDPOLLEX` / `IO_ENABLEPOLL` / `IO_CFG` / `IO_DECODE` (IOC TX points)
- `IOC RSP` (response logs used to confirm which IOC steps succeeded/failed)
- `RAKHUB` (USB CDC command entry: `RS485/AIC/BUILTIN/APPLY/CLEARAIC`, etc.)
- **`rakhubUsbFeedByte()`** / **`parseCo2Ipso()`** (USB line assembly; IPSO 0x7D CO₂ ppm)

#### 3.1b USB + protobuf coexistence

- **`src/mesh/StreamAPI.cpp`**: when hunting Meshtastic protobuf `START1` (`0x94`), non-matching bytes are forwarded to **`rakhubUsbFeedByte()`** (declared in `RAKSensorHubProfile.h`). Without this, `RAKHUB APPLY` lines are dropped if any protobuf client touched the port first.

#### 3.2 Python helper (minimal USB text sender)

- **`bin/rakhub_usb_poc.py`**
  - `rs485` / `aic` / `builtin` / `json`: send `RAKHUB ...` lines
  - `clear`: send `RAKHUB BUILTIN 0` + `RAKHUB APPLY` (clear RS485/IOC defaults)
  - `clearaic`: send `RAKHUB CLEARAIC` (clear AIC decode defaults)
  - `json --apply`: WisToolBox-style JSON → RAKHUB lines; **always emits `slot=i`** per task (clears stale USB template RAM)
  - Verified GE profiles: `bin/battery_lite_core-1.2.27.json` (RAK9154), `bin/rk300_03_core-1.2.27.json` (RK300-03 CO₂)

#### 3.3 Docs

- `doc/RAKSensorHub_Architecture_en.md`: architecture + uplink data path (also mentions the downlink POC)
- `doc/README_RAKSensorHub_Downlink_POC.en.md`: downlink POC handoff (commands, caveats, file list)
- `README(Add_RAKSensorHub).md`: project entry + documentation map
- `doc/RAKSensorHub_Uplink_Sensor_Support_List.en.md`: uplink support list (IPSO → telemetry fields)

---

### 4) What “IOC verification” means (how to read logs)

During downlink provisioning, the Hub sends IOC commands in sequence and logs lines like:

- `IOC TX: IO_ADDPOLLEX task=...`
- `IOC RSP: func=IO_ADDPOLLEX ...`

Rule of thumb:

- If you see **TX** but not the corresponding **RSP**, the step was likely not confirmed (often alongside `+ERR:CHKSUM / +ERR:SEQUCE / +ERR:OVF`).
- Verification means: **each key IOC command receives the expected RSP**, and ProbeIO starts reporting the expected IPSOs for the configured tasks.

---

### 5) What the USB / Python CLI is (and is not)

This is not a production configuration channel. It is a lab / POC helper:

- **USB CDC (firmware side)**: parses `RAKHUB ...` text, stores a USB override template, then `APPLY` triggers a downlink POC run (clear → wait for ProbeIO power-cycle/rejoin → config).
- **Python (PC side)**: sends those text lines over the serial port and can convert JSON → RAKHUB lines.

Note: do not share the same CDC port with a Meshtastic protobuf client while sending `RAKHUB ...` lines. If a protobuf client already hunted `START1`, leftover ASCII is still forwarded via `rakhubUsbFeedByte()` (see §3.1b) — but exclusive port use remains the reliable lab rule.

---

### 6) How to use the Python helper (`bin/rakhub_usb_poc.py`)

#### 6.1 Prerequisites

- Install dependency:

```bash
python -m pip install pyserial
```

- Use the Probe/Hub CDC port (example: `COM3`).
- Disconnect any other tool that might hold the same port (Meshtastic app, serial monitors, etc.).

#### 6.2 Common command patterns

- **Print hub-side status** (shows template selection / flags):

```bash
python bin/rakhub_usb_poc.py --port COM3 status
```

- **Apply one WisToolBox-style JSON template** (RS485 multi-task supported):

```bash
python bin/rakhub_usb_poc.py --port COM3 --pid 01 --apply json --file bin/wistool_SDSIN_SOIL_4IN1.json
```

- **Clear RS485/IOC defaults on ProbeIO** (built-in clear-only template + apply):

```bash
python bin/rakhub_usb_poc.py --port COM3 --pid 01 clear
```

- **Clear AIC decode defaults** (does not require `--apply`):

```bash
python bin/rakhub_usb_poc.py --port COM3 --pid 01 clearaic
```

#### 6.3 When ProbeIO needs a power-cycle (expected in POC)

ProbeIO power-cycle/rejoin is typically needed when:

- You run `clear` (because the downlink POC intentionally enters “wait for rejoin” before the config phase), or
- You change the RS485 task set significantly (e.g. switching templates), and you see missing `IOC RSP` confirms.

Typical lab sequence:

- `clear` → power-cycle ProbeIO → wait for `provision=1` / PID rejoin → `json --apply ...` → verify `IOC RSP` confirms and IPSO reports.

---

### 7) Snapshot: latest git context (for alignment)

Latest commit on this branch (HEAD):

- `bcd835654`：GE JSON downlink — `DOWNLINK_AUTO=0`, `StreamAPI`→`rakhubUsbFeedByte`, `battery_lite` / `rk300_03` JSON, CO₂ parse + log throttling (see `doc/RAKSensorHub_CHANGELOG.md` §〇-B)

Key commits related to the downlink POC:

- `3e93e8fc8`: downlink IOC POC (`IO_ADDPOLLEX`, etc.)
- `f12f0819a`：AIC `IO_DECODE` downlink
- `d79f2d62a`: USB text POC + `bin/rakhub_usb_poc.py` (multi-task `slot=` approach)
- `bcd835654`: GE JSON APPLY path + USB/protobuf coexistence + CO₂ uplink verified

To list changes since the uplink baseline tag:

- `git diff --name-only v2.7.21-raksensorhub-poc-uplink..HEAD`
- `git log --oneline v2.7.21-raksensorhub-poc-uplink..HEAD`

