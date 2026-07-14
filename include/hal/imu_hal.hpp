#pragma once

#include <cstdint>
#include <Arduino.h>
#include <SPI.h>

struct IMUReading {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    // Native 16-bit sensor counts (before scaling) - used for telemetry downlink
    // so the full sensor resolution is preserved and the ground station applies
    // the documented scale factors.
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
    uint32_t timestamp;
    bool valid;
};

class IMUHAL {
public:
    IMUHAL(uint8_t cs_accel, uint8_t cs_gyro, uint8_t spi_port = 0);

    bool init();
    bool read(IMUReading& reading);
    void calibrate(uint16_t samples = 100);
    bool selfTest();

private:
    uint8_t cs_accel_;
    uint8_t cs_gyro_;
    SPIClass* spi_;
    bool initialized_ = false;

    uint8_t accelRead(uint8_t reg);
    void accelWrite(uint8_t reg, uint8_t data);
    void accelReadBytes(uint8_t reg, uint8_t* buf, uint8_t len);

    uint8_t gyroRead(uint8_t reg);
    void gyroWrite(uint8_t reg, uint8_t data);
    void gyroReadBytes(uint8_t reg, uint8_t* buf, uint8_t len);
};
