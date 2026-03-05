#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(HAS_RAKHUB)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "CurrentSensor.h"
#include "TelemetrySensor.h"
#include "VoltageSensor.h"

// 单值传感器读数：数值 + 有效标志 + 最后更新时间（便于扩展新 IPSO 类型）
struct ScalarReading {
    float value = 0.0f;
    uint32_t lastUpdateMs = 0;
    bool valid = false;
};

// 三轴传感器（如加速度计）
struct AccelReading {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    uint32_t lastUpdateMs = 0;
    bool valid = false;
};

// 电源模块读数（RAK9154 等，来自 IPSO 0xB8/0xB9/0xBA）：母线电压、电流、电量百分比
struct HubPower {
    uint16_t volMv = 0;    // 母线电压 mV
    int16_t curMa = 0;     // 母线电流 mA
    uint8_t percent = 0;   // 电量 0..100
};

/*
IPSO 类型定义见 onewire_master_protocol.h（https://github.com/beegee-tokyo/RAK-OneWireSerial/blob/main/src/onewire_master_api.h）
*/
// 环境传感器缓存：所有 IPSO 类型统一在此，新增传感器只需加字段
struct EnvCache {
    ScalarReading temperature;     // 0x67 温度 °C
    ScalarReading humidity;        // 0x68 湿度 %
    ScalarReading pressure;        // 0x73 气压 hPa
    ScalarReading soil_moisture;   // 0x70 高精度湿度 %
    ScalarReading wind_speed;      // 0xBE 风速 m/s
    ScalarReading wind_direction;  // 0xBF, 0..360 风向 °
    ScalarReading radiation;       // 0xC3 辐射 W/m²
    ScalarReading soil_ph;         // 0xC1 土壤 pH
    ScalarReading salinity;        // 0x13 盐度
    ScalarReading ec;             // 0xC0 电导率 μS/cm
    AccelReading accel;           // 0x71 加速度 m/s²
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