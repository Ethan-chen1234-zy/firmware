# RAKSensorHub — Current Uplink Sensor Support (Reference)

---

## 1. WisBlock modules (direct on probe)

| Module | Chip | Quantity | IPSO | Format | Notes |
|--------|------|----------|------|--------|-------|
| **RAK1901** | SHTC3 | Temperature | 0x67 | 2-byte signed, 0.1 °C | Probe types A, D, E, J |
| | | Humidity | 0x68 | 1 byte, 1 %RH | |
| **RAK1902** | LPS22HB | Temperature | 0x67 | 2-byte signed, 0.1 °C | Probe types B, D, F, I |
| | | Pressure | 0x73 | 2-byte unsigned, 0.1 hPa | |
| **RAK1904** | LIS3DH | 3-axis accel | 0x71 | 6 bytes, 2 bytes/axis signed, 0.001 g | Probe types C, E, F, H |

---

## 2. Prebuilt third-party sensors (in firmware)

Shipped in SensorHub firmware; use with ProbeIO directly.

| SKU | Model | Type | Bus | Quantity | Register | Scale | Unit | IPSO |
|-----|-------|------|-----|----------|------------|-------|------|------|
| **RA** | RK900-09 | Weather station | RS485 | Wind speed | 0x0000 | 0.01 | m/s | 0xBE |
| | | | | Wind dir | 0x0001 | 1 | ° | 0xBF |
| | | | | Temperature | 0x0002 | 0.1 | °C | 0x67 |
| | | | | Humidity | 0x0003 | 0.1 | %RH | 0x70 |
| | | | | Pressure | 0x0004 | 0.1 | hPa | 0x73 |
| **RB** | RK520-02 | Soil multi-param | RS485 | Temperature | 0x0000 | 0.1 | °C | 0x67 |
| | | | | Moisture | 0x0001 | 0.1 | % (m³/m³) | 0xBC |
| | | | | EC | 0x0002 | 0.001 | mS/cm | 0xC0 |
| **RC** | JXBS-3001-pH | Soil pH | RS485 | High-res pH | 0x0006 | 0.01 | pH | 0xC1 |
| | | | | Low-res pH | 0x000d | 0.1 | pH | 0xC2 |
| **RD** | UTOP ULB16 | Level | 4–20 mA | Level | — | — | mm | 0x82 |
| **RE** | RK200-03 | Pyranometer | SDI-12 | Irradiance | — | 1 | W/m² | 0xC3 |

---

## 3. Downstream / generic sensors (target scope)

### 3.1 RS485 (Modbus RTU) — generic integration

Registers and datatype are per sensor manual. Table below: typical agri/water mappings (examples).

| Quantity | Typical register | Length | Scale | Unit | IPSO | Notes |
|----------|------------------|--------|-------|------|------|-------|
| Nitrogen | User-defined | 2 | 1 | mg/kg | 0x10 | |
| Phosphorus | User-defined | 2 | 1 | mg/kg | 0x11 | |
| Potassium | User-defined | 2 | 1 | mg/kg | 0x12 | |
| Salinity | User-defined | 2 | 0.01 | mg/L | 0x13 | |
| Dissolved O₂ | User-defined | 2 | 0.01 | mg/L | 0x14 | |
| ORP | User-defined | 2 | 0.1 | mV | 0x15 | |
| COD | User-defined | 2 | 1 | mg/L | 0x16 | |
| Turbidity | User-defined | 2 | 1 | NTU | 0x17 | |
| Nitrate (NO₃) | User-defined | 2 | 0.1 | ppm | 0x18 | |
| Ammonium (NH₄⁺) | User-defined | 2 | 0.01 | ppm | 0x19 | |
| BOD | User-defined | 2 | 1 | mg/L | 0x1A | |
| High-precision EC | User-defined | 4 | 0.001 | µS/cm | 0x7F | |
| Distance | User-defined | 4 | 0.001 | m | 0x82 | |
| PM10 | User-defined | 2 | 1 | µg/m³ | 0xE3 | |
| PM2.5 | User-defined | 2 | 1 | µg/m³ | 0xE4 | |
| Noise | User-defined | 2 | 0.1 | dB | 0xE9 | |

### 3.2 SDI-12

| Quantity | Example | IPSO | Notes |
|----------|---------|------|-------|
| Irradiance | RK200-03 (SDI-12) | 0xC3 | W/m² direct |
| Level | Other SDI-12 level sensors | 0x82 | Needs configuration |

### 3.3 Analog (4–20 mA / 0–5 V)

| Quantity | Example | Interface | Notes |
|----------|---------|-----------|-------|
| Level | UTOP ULB16 (4–20 mA) | 4–20 mA | Resistor to voltage |
| Pressure | 4–20 mA transmitter | 4–20 mA | Linear scaling |
| Light | 0–5 V lux sensor | 0–5 V | External conditioning (not first-class in official list) |

---

## 4. EnvCache ↔ `EnvironmentMetrics` (Sprint 1 / D1-5, P1 farm)

| EnvCache | IPSO | Proto field | Field # | Unit / notes |
|----------|------|-------------|---------|----------------|
| `ec` | 0xC0, 0x7F | `soil_conductivity` | 24 | mS/cm (`env.ec`; 0x7F uses µS/cm scale in parser) |
| `soil_ph` (fallback `ph`) | 0xC1, 0xC2 | `soil_ph` | 25 | pH |
| `salinity` | 0x13 | `salinity` | 26 | mg/L |
| `digital_input` | 0x00 | `digital_input` | 27 | 0/1 |
| `digital_output` | 0x01 | `digital_output` | 28 | 0/1 |
| `nitrogen` | 0x10 | `soil_nitrogen` | 29 | mg/kg, scale 1 |
| `phosphorus` | 0x11 | `soil_phosphorus` | 30 | mg/kg, scale 1 |
| `potassium` | 0x12 | `soil_potassium` | 31 | mg/kg, scale 1 |
| `dissolved_oxygen` | 0x14 | `dissolved_oxygen` | 32 | mg/L, scale 0.01 |
| `orp` | 0x15 | `orp` | 33 | mV, scale 0.1 (int16) |
| `cod` | 0x16 | `cod` | 34 | mg/L, scale 1 |
| `turbidity` | 0x17 | `turbidity` | 35 | NTU, scale 1 |
| `nitrate` | 0x18 | `nitrate` | 36 | ppm, scale 0.1 |
| `ammonium` | 0x19 | `ammonium` | 37 | ppm, scale 0.01 |
| `bod` | 0x1A | `bod` | 38 | mg/L, scale 1 |
| `moisture` | 0xBC | `soil_moisture` | (existing) | %, scale 0.1 → uint8 0..100 |

Parsed in `onewire_evt()` SDATA/REPORT `switch` (same style as salinity/EC/pH). Exported in `RAKSensorHub::getMetrics()` when cache age &lt; 5 min. Regen: `./bin/regen-protos.sh` (or patch `telemetry.pb.h` on Windows without nanopb bundle).

---

## 5. IPSO quick reference (partial)

| IPSO | Dec | Description |
|------|-----|-------------|
| 0x67 | 103 | Temperature |
| 0x68 | 104 | Humidity (1 % steps) |
| 0x70 | 112 | High-precision humidity (0.1 %) |
| 0x73 | 115 | Barometric pressure |
| 0x71 | 113 | Accelerometer |
| 0xBE | 190 | Wind speed |
| 0xBF | 191 | Wind direction |
| 0xC0 | 192 | Electrical conductivity |
| 0xC1 | 193 | High-precision pH |
| 0xC2 | 194 | Standard pH |
| 0xC3 | 195 | Pyranometer |
| 0xBC | 188 | Soil moisture |

---

## 6. Summary

RAK2560 SensorHub connects **multiple probes** over **OneWire**; each probe can host **1–2 WisBlock modules**, or **ProbeIO** can expose **RS485, SDI-12, 4–20 mA** for third-party sensors. Firmware embeds Modbus register maps and IPSO definitions for common third-party devices. **Current uplink build does not support user provisioning of new sensors** — that requires the **downlink/config** track.

---

## Appendix: figures (same assets as Chinese doc)

Use paths relative to this file:

- Grafana: `./POC-Grafana.png`
- Meshtastic App: `./POC-Meshtastic-APP.png`
- RAKSensorHub + Probe: `./RAKSensorHub+Probe.png`
- RAKSensorHub + ProbeIO: `./RAKSensorHub+ProbeIO2.jpg`, `./RAKSensorHub+ProbeIO.jpg`, `./RAKSensorHub+ProbeIO3.jpg`, `./RAKSensorHub+ProbeIO4.jpg`
- Meshtastic context: `./在Meshtastic中的位置.png`
- Misc: `./RAK-micro-rgb.png`
- Architecture: `./SensorHub架构.png`

**Parameter grid (marketing-style ranges)** — same intent as Chinese tables:

| Temp −40 °C ~ 125 °C | Humidity 0 ~ 100 % | Pressure 260 ~ 1260 hPa | 3-axis accel ±16 g |
|----------------------|----------------------|-------------------------|----------------------|
| Wind 0 ~ 40 m/s | Wind dir 0 ~ 360° | Solar 0 ~ 2000 W/m² | CO₂ 0 ~ 5000 ppm |
| Soil moisture 0 ~ 100 % | pH 0 ~ 14 (0.01) | EC 0 ~ 10 mS/cm | Level 0 ~ 5 m |

**ProbeIO + prebuilt third-party**

| Sensor | Interface | Metrics |
|--------|-----------|---------|
| RK900-09 weather | RS485 | Wind speed/dir, temp, humidity, pressure |
| RK300-03 | RS485 | CO₂ |
| JXBS-3001-pH | RS485 | pH (hi/lo res) |
| UTOP ULB16 | 4–20 mA | Level |
| RK200-03 | SDI-12 | Solar irradiance |

**Probe + WisBlock**

| Module | Built-in bus | Metrics |
|--------|----------------|---------|
| RAK1901 | I²C | Temp + humidity |
| RAK1902 | I²C | Temp + pressure |
| RAK1904 | I²C | 3-axis accel |

| Probe + WisBlock | Built-in | Metrics |
|------------------|----------|---------|
| RAK1901 | I²C | Temp + humidity |
| RAK1902 | I²C | Temp + pressure |
| RAK1904 | I²C | 3-axis accel |
| **ProbeIO + prebuilt** | **Interface** | **Metrics** |
| RK900-09 weather | RS485 | Wind, dir, temp, humidity, pressure |
| RK520-02 soil | RS485 | Temp, moisture, EC |
| JXBS-3001-pH | RS485 | pH (hi/lo) |
| RK200-03 | SDI-12 | Solar irradiance |
| UTOP ULB16 | 4–20 mA | Level |
