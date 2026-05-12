## RAKSensorHub architecture and behavior in Meshtastic

This document describes the design, architecture, and runtime behavior of `RAKSensorHub` as implemented in
`src/modules/Telemetry/Sensor/RAKSensorHub.cpp` / `.h`, and how it integrates with the Meshtastic firmware.

The focus is **uplink telemetry only** (reading sensors via RAK Probe IO / SensorHub). Downlink configuration
of Probe IO / RS485 devices is intentionally out of scope for the current firmware snapshot.

---

### 1. High‑level role

`RAKSensorHub` is a `TelemetrySensor` implementation that:

- Talks to **RAK Probe IO / SensorHub** over a 1‑Wire‑style UART (SoftwareHalfSerial).
- Receives frames encoded using **RAK’s OneWire protocol** (with IPSO object IDs).
- Parses those frames into an internal **environment cache** (`EnvCache`) and **power cache** (`HubPower`).
- Exposes sensor readings to Meshtastic as:
  - `EnvironmentMetrics` (environment telemetry module), and
  - `AirQualityMetrics` (air quality telemetry module, for CO₂).

It is not an I²C sensor; instead it is always “present” on builds with `HAS_RAKHUB` defined, and relies on
Probe IO to aggregate RAK190x modules and RS485/SDI‑12 external sensors.

---

### 2. Data model: caches and types

#### 2.1 `ScalarReading` and `AccelReading`

To accommodate many IPSO types and to handle “latest valid value + freshness window”, the code uses:

- `ScalarReading`:
  - `float value`
  - `uint32_t lastUpdateMs`
  - `bool valid`
- `AccelReading`:
  - `float x, y, z`
  - `uint32_t lastUpdateMs`
  - `bool valid`

Two helpers operate on these:

- `setScalar(ScalarReading &r, float v, uint32_t nowMs)`  
  Updates value, marks as valid, and stamps the last update time.
- `scalarFresh(const ScalarReading &r, uint32_t nowMs, uint32_t maxAgeMs)`  
  Returns true if the reading is valid, timestamp is non‑zero, and age ≤ `maxAgeMs`.

This allows the hub to smooth over missed frames and still provide reasonably fresh telemetry to the app.

#### 2.2 `HubPower`

`HubPower` stores RAK9154 / power‑module related values:

- `uint16_t volMv` – bus voltage in mV (IPSO 0xBA, 0.01 V units)
- `int16_t curMa` – bus current in mA (IPSO 0xB9, 0.01 A units)
- `uint8_t percent` – battery capacity 0..100 % (IPSO 0xB8)

These are exported to telemetry via `getBusVoltageMv()`, `getCurrentMa()`, `getBusBatteryPercent()` and
ultimately mapped into `EnvironmentMetrics.voltage` and `.current`.

#### 2.3 `EnvCache`

`EnvCache` is the central environment cache where all IPSO readings are accumulated:

- `temperature` (0x67, °C)
- `humidity` (0x68, %RH)
- `pressure` (0x73, hPa)
- `high_precision_humidity` (0x70, 0.1 % – used as high‑precision humidity for soil or weather)
- `wind_speed` (0xBE, m/s)
- `wind_direction` (0xBF, 0..359°)
- `radiation` (0xC3, W/m²)
- `soil_ph` (0xC1)
- `salinity` (0x13)
- `ec` (0xC0, mS/cm)
- `co2` (0x7D, ppm)
- `accel` (0x71, 3‑axis accelerometer)

This single cache is filled whenever IPSO sensor frames are parsed and later read out in `getMetrics()`.

---

### 3. Threading and scheduling model

Meshtastic uses a cooperative “pseudo‑threading” model based on `OSThread` and `ThreadController`. Within that:

- `RAKSensorHub` itself is a `TelemetrySensor` with a `runOnce()` method.
- On the **first** call to `RAKSensorHub::runOnce()`:
  - It constructs two `Periodic` `OSThread`s:
    - `onewireRxPeriodic` → runs `onewireRxHandle()`
    - `onewirePollPeriodic` → runs `onewirePollHandle()`
  - Initializes the 1‑Wire UART (`mySerial.begin(9600);`).
  - Initializes the RAK OneWire protocol stack: `RakSNHub_Protocl_API.init(onewire_evt);`
  - Sets internal `initialized` flag.
- Subsequent `runOnce()` calls simply return a default sensor interval; ongoing work is done by the two
  `Periodic` threads.

Main loop (`mainController.runOrDelay()`) drives all `OSThread` instances in a cooperative way. Therefore,
`onewireRxHandle()` and `onewirePollHandle()` must be short‑running and frequently rescheduled to keep up
with 9600 bps half‑duplex traffic.

---

### 4. OneWire protocol integration

#### 4.1 `onewire_evt` – protocol event callback

`onewire_evt(const uint8_t pid, const uint8_t sid, const SNHUBAPI_EVT_E eid, uint8_t *msg, uint16_t len)`
is registered via `RakSNHub_Protocl_API.init(onewire_evt)` and called by the protocol library after a
frame is parsed.

It handles:

- `SNHUBAPI_EVT_RECV_REQ` / `RECV_RSP`: request/response completion and PID advancement.
- `SNHUBAPI_EVT_QSEND`: actual UART TX (writes bytes to `mySerial` and sets `awaiting_rsp` state).
- `SNHUBAPI_EVT_ADD_PID` / `ADD_SID`: Probe “registration”, updating `provision_list` and `sid_seen`.
- `SNHUBAPI_EVT_SDATA_REQ` / `REPORT`:  
  - `msg[0]` is the IPSO type (0x67, 0x68, 0x70, 0x73, 0x7D, 0xBE, 0xBF, 0xC3, 0xC0, etc.).  
  - The switch dispatches to appropriate parsing code and updates `env` / `power` with `setScalar()`.  
  - Includes range checks and filters to drop obviously invalid readings.  
  - Marks `lastRead` for the `TelemetrySensor` so upper layers know the hub was active recently.
- `SNHUBAPI_EVT_CHKSUM_ERR` / `SEQ_ERR`:  
  - Marks `frame_error_during_process`, sets `last_err_time`, clears `awaiting_rsp`.  
  - RX handler then discards or resyncs the buffer to recover from corrupted frames.

This callback is the bridge between **low‑level protocol** (bytes → SNHUB API) and the **high‑level
environment/power caches**.

#### 4.2 `onewireRxHandle()` – RX periodic task

Responsibilities:

- Drain `mySerial` as fast as possible into `buff[]`.
- Detect software UART **overflow**:
  - If `mySerial.overflow()` → logs `+ERR:OVF`, resets buffer, sets `last_err_time`, and opens a short
    “RX only” window by extending `listen_window_until`.
- Frame assembly:
  - Aligns to `DELIMTER` (0x7E) start bytes.
  - Uses the embedded length (`buff[1..2]`) to compute full frame size.
  - Waits until enough bytes have been collected (`bufflen >= total_needed`).
  - Handles insane lengths by dropping one byte and resyncing.
- Capability fallback:
  - If frame type `buff[3]` is `0x45` or `0x4D` (capability frame), it:
    - Extracts PID from `buff[34]` (or `buff[33]` / a new free PID if `0xFF`).
    - Inserts the PID into `provision_list` if not already present.
    - Logs `+BOOT:PID[...] from capability`.
    - Opens a short listen window (no TX) to avoid colliding with join traffic.
- Hand off to RAK protocol:
  - Calls `RakSNHub_Protocl_API.process(buff, total_needed)`.
  - Uses `frame_error_during_process` to decide whether to drop one byte and resync.
  - On success, removes exactly the processed frame from the buffer.
- Scheduling:
  - Returns a short interval (5 ms) when activity is high, and 40 ms when idle.

This function is deliberately tight and I/O focused: it must keep up with 9600 bps frames in a cooperative
threading environment without starving other tasks.

#### 4.3 `onewirePollHandle()` – poll / discovery periodic task

Responsibilities:

- Periodic **listen windows**:
  - Every 60s, it schedules a 3s “no TX” window (`listen_window_until`) to let new probes send
    capability/join frames without collision.
- Post‑capability quiet time:
  - If `provision_list` is empty and a capability was seen <1.5s ago, it defers TX and lets RX handle
    follow‑up frames.
- Link idle detection:
  - Ensures TX only happens when no bytes have been received for ≥10 ms and it has been ≥40 ms since last TX.
- Outstanding request management:
  - Ensures there is at most one outstanding request (half‑duplex safety).
  - If no response in >2000 ms, it clears `awaiting_rsp`, logs an error timestamp, and advances the poll PID.
- Status logging:
  - Every ~5s prints a status line containing `last_tx`, `last_rx`, and `provision_list` size.
- Discovery:
  - When `provision_list` is empty (or periodically when not empty), it uses `RakSNHub_Protocl_API.get.data()`
    on PIDs 0x01..0x04 to trigger probe responses and registration.
- Round‑robin polling:
  - Once probes are provisioned, it polls one PID at a time, with a ~1.5s interval, and a slower 5s “fallback”
    poll to keep sensors warm.
- BLE‑aware throttling (current snapshot):
  - If BLE/phone is connected (`service->api_state == MeshService::STATE_BLE`) **and** `provision_list` is empty
    (still in join phase), it extends listen windows and defers discovery TX. This reduces join failures caused
    by BLE/CPU load.

In summary, `onewirePollHandle()` primarily decides **when to send** `get.data()` and when to stay quiet, while
`onewireRxHandle()` focuses on **what to do with bytes coming in**.

---

### 5. Integration with Meshtastic telemetry

`RAKSensorHub` integrates with the Meshtastic telemetry modules as follows:

- `RAKSensorHub` is included by `EnvironmentTelemetry` and `AirQualityTelemetry` when `HAS_RAKHUB` is defined.
- `EnvironmentTelemetryModule` and `AirQualityTelemetryModule` call:
  - `rakSensorHub.runOnce()` during initialization (to set up the threads and UART).
  - `rakSensorHub.getMetrics(&m)` when assembling telemetry packets.

#### 5.1 `getMetrics()` – Environment & CO₂ metrics

`bool RAKSensorHub::getMetrics(meshtastic_Telemetry *measurement)`:

- Uses a `maxAgeMs` window (5 minutes) to decide whether cached `env` values are still “fresh enough”.
- If `measurement->which_variant == meshtastic_Telemetry_air_quality_metrics_tag`:
  - It only fills `AirQualityMetrics.co2` if `env.co2` is fresh.
- Otherwise, it fills `EnvironmentMetrics`:
  - `voltage` / `current` from `HubPower` if voltage > 0.
  - `temperature` / `relative_humidity` / `barometric_pressure` from `env`.
  - `wind_speed` / `wind_direction`.
  - High‑precision humidity (0x70):
    - If `high_precision_humidity` is fresh, it:
      - Exposes it as `relative_humidity` (if plain humidity is not already set).
      - Exposes it as `soil_moisture` (clamped 0..100).
  - `radiation` from pyranometer (W/m²).
- Returns `true` if any field was filled, so the caller can OR this with other sensors’ results.

This approach allows multiple sensors (I²C + RAKHub) to contribute to a single `EnvironmentMetrics` packet in
an additive way.

---

### 6. Design goals and trade‑offs

Key design goals:

1. **Support a wide range of RAK sensors** via a single OneWire/Probe IO integration.
2. **Map everything into standard Meshtastic telemetry structs** so existing app / tooling can consume the data.
3. **Be robust to timing jitter and CPU load**, especially when the phone app is connected over BLE.
4. **Avoid hard coupling to a single RS485/Modbus configuration** – leave configuration to Probe IO tools.

Main trade‑offs:

- Continued reliance on `SoftwareHalfSerial` and cooperative scheduling makes the RX path sensitive to load.
  The code mitigates this with:
  - Short RX intervals (5 ms active).
  - Overflow detection and resync.
  - Listen windows and TX throttling on capability/join.
  - Conservative freshness windows (5 minutes) for env cache.
- Commands / downlink (e.g. RS485 register configuration, PARAMSET frames) are **not implemented**, to keep
  the hub focused on reliable uplink telemetry in this snapshot. Future work may add:
  - A command queue layered over `RakSNHub_Protocl_API`.
  - Admin/Serial APIs for sending specific Probe IO commands.
  - Further decoupling of RX, TX, and BLE activity.

---

### 7. Summary

`RAKSensorHub` is a bridge between:

- The **RAK OneWire / Probe IO world** (IPSO frames over a 1‑Wire half‑duplex UART), and
- Meshtastic’s **Environment/Air quality telemetry**.

It:

- Maintains a robust stateful cache of environment and power readings.
- Uses two cooperative periodic tasks to:
  - Drain and parse UART RX.
  - Schedule polls/discovery with hot‑plug and BLE load in mind.
- Exposes a simple `getMetrics()` API that other telemetry modules call to decorate standard
  `meshtastic_Telemetry` messages.

This architecture makes it possible to plug a wide range of RAK SensorHub ecosystems into Meshtastic
with minimal app changes, while keeping the firmware side confined to a single, well‑defined module.

