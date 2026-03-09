#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(HAS_RAKHUB)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "CurrentSensor.h"
#include "TelemetrySensor.h"
#include "VoltageSensor.h"

// Single-value sensor reading: value + valid flag + last update time (for extending new IPSO types)
struct ScalarReading {
    float value = 0.0f;
    uint32_t lastUpdateMs = 0;
    bool valid = false;
};

// 3-axis sensor (e.g. accelerometer)
struct AccelReading {
    float x = 0.0f, y = 0.0f, z = 0.0f;  
    uint32_t lastUpdateMs = 0;  
    bool valid = false;
};

// Power module reading (RAK9154 etc., from IPSO 0xB8/0xB9/0xBA): bus voltage, current, capacity percent
struct HubPower {
    uint16_t volMv = 0;    // 0xB9 Bus voltage mV
    int16_t curMa = 0;     // 0xB8 Bus current mA
    uint8_t percent = 0;   // 0xBA Capacity 0..100
};

/*
 * IPSO type definitions: onewire_master_protocol.h
 * (https://github.com/beegee-tokyo/RAK-OneWireSerial/blob/main/src/onewire_master_api.h)
 */
// Environment sensor cache: all IPSO types in one place; add a field when adding a new sensor
struct EnvCache {
    ScalarReading temperature;     // 0x67 temperature °C
    ScalarReading humidity;       // 0x68 humidity %
    ScalarReading pressure;       // 0x73 pressure hPa
    ScalarReading soil_moisture;  // 0x70 high-precision humidity %
    ScalarReading wind_speed;     // 0xBE wind speed m/s
    ScalarReading wind_direction; // 0xBF wind direction 0..360 °
    ScalarReading radiation;      // 0xC3 radiation W/m²
    ScalarReading soil_ph;        // 0xC1 soil pH
    ScalarReading salinity;       // 0x13 salinity
    ScalarReading ec;             // 0xC0 conductivity μS/cm
    ScalarReading co2;            // 0x7D co2 ppm
    AccelReading accel;           // 0x71 accelerometer m/s²
};

class RAKSensorHub : public TelemetrySensor, VoltageSensor, CurrentSensor
{
  private:
  protected:
    virtual void setup() override;
    uint32_t lastRead = 0;

  public:
    RAKSensorHub();
    bool hasSensor() { return true; } // Not an I2C sensor; always available when DHAS_RAKHUB is defined
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual uint16_t getBusVoltageMv() override;
    virtual int16_t getCurrentMa() override;
    int getBusBatteryPercent();
    bool isCharging();
    void setLastRead(uint32_t lastRead);
};
extern RAKSensorHub rakSensorHub;

#endif // HAS_RAKHUB