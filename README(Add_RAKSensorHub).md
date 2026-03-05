# RAK SensorHub Integration for Meshtastic

This branch (feature/add_RAKSensorHub) contains the ongoing work to integrate RAK's SensorHub ecosystem into the Meshtastic firmware. The goal is to enable Meshtastic devices to read data from a wide variety of RAK sensors and probes via the OneWire protocol, and report them as standard Meshtastic telemetry.

### 📌 Current Status

Core Protocol: The OneWire communication layer with RAK Probe IO and Sensor Probes is functional.

Sensor Support: Successfully tested with multiple sensor types:

RAK1901: Temperature & Humidity

RAK1902: Barometric Pressure

RAK1904: 3-axis Accelerometer

RAK9154: Battery voltage, current, capacity, temperature

RS485 Soil Sensors: High-precision humidity, temperature, salinity, EC

RK300-03: CO₂ concentration

RK900-09 Weather Station: Wind speed, wind direction, temperature, humidity, pressure

RK200-03: Solar radiation (Pyranometer) via SDI-12

Data Flow: Parsed sensor data is successfully mapped to the Meshtastic EnvironmentMetrics structure and can be viewed in the app.

### ⚠️ Important Notes for This Branch

Configuration via WisToolBox: The current Meshtastic firmware build in this branch does not support configuring RAK hardware (like Probe IO) through the WisToolBox software. Any AT command configuration must be handled differently.

Probe IO Configuration in Progress: Methods for configuring Probe IO (e.g., setting RS485 baud rates, adding Modbus polling tasks) are currently being tested. This is a work in progress to find the most reliable way to set up the hardware without WisToolBox.

Development Phase: This is an active development branch. While core sensor reading is stable, configuration workflows and support for all possible RAK hardware combinations are still being validated.

### 🧪 Hardware Tested

Hub: RAK2560 (running this Meshtastic firmware)

IO Board: Probe IO (for RS485 and SDI-12 sensors)

Probes: RAK1901, RAK1902, RAK1904

External Sensors: RK300-03 (CO₂, RS485), RK900-09 (Weather Station, RS485), RK200-03 (Pyranometer, SDI-12), Various soil sensors (RS485)

### 🚀 Getting Started (for Testers)

Flash the Firmware: Build and flash the firmware from this branch onto your RAK2560 device.

```
pio run -e rak2560 --target upload
```

Connect Hardware: Attach your RAK probes or Probe IO with external sensors to the RAK2560.

Power Up: Ensure the system is powered correctly (Probe IO and some external sensors require 12V).

Observe Telemetry: Connect to the device via the Meshtastic app. You should see environmental telemetry data appearing from the connected sensors.

### 🤝 How to Help / Provide Feedback

Your testing and feedback are invaluable! If you encounter issues or have suggestions:

Report Bugs: Please open an issue on this repository with details about your hardware setup and the problem observed.

Configuration Insights: If you have experience configuring Probe IO without WisToolBox (e.g., using direct serial commands), please share your findings!

Sensor Compatibility: Let us know which RAK sensors or third-party probes you've tested and whether they worked.

We are actively working on stabilizing the configuration methods and will provide updates as progress is made.

Feel free to modify the tone and details to better fit your style. This draft provides a clear and honest overview of your branch's current state.
