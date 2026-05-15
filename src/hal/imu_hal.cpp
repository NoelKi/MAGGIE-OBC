#include "hal/imu_hal.hpp"

// BMI088 Accelerometer registers
static constexpr uint8_t ACC_CHIP_ID     = 0x00;
static constexpr uint8_t ACC_DATA_START  = 0x12;
static constexpr uint8_t ACC_CONF        = 0x40;
static constexpr uint8_t ACC_RANGE       = 0x41;
static constexpr uint8_t ACC_PWR_CONF    = 0x7C;
static constexpr uint8_t ACC_PWR_CTRL    = 0x7D;
static constexpr uint8_t ACC_SOFTRESET   = 0x7E;

static constexpr uint8_t ACC_CHIP_ID_VAL = 0x1E;
static constexpr uint8_t ACC_RANGE_3G    = 0x00;
// ODR 100Hz, normal bandwidth
static constexpr uint8_t ACC_CONF_VAL    = 0xA8;

// BMI088 Gyroscope registers
static constexpr uint8_t GYR_CHIP_ID     = 0x00;
static constexpr uint8_t GYR_DATA_START  = 0x02;
static constexpr uint8_t GYR_RANGE       = 0x0F;
static constexpr uint8_t GYR_BANDWIDTH   = 0x10;
static constexpr uint8_t GYR_LPM1        = 0x11;
static constexpr uint8_t GYR_SOFTRESET   = 0x14;

static constexpr uint8_t GYR_CHIP_ID_VAL  = 0x0F;
static constexpr uint8_t GYR_RANGE_500DPS = 0x02;
// 100Hz ODR, 32Hz filter
static constexpr uint8_t GYR_BW_VAL       = 0x07;

// Scale factors
static constexpr float ACCEL_SCALE = 1.0f / 10920.0f * 9.80665f; // ±3g → m/s²
static constexpr float GYRO_SCALE  = 1.0f / 65.536f;              // ±500°/s → °/s

static constexpr uint32_t SPI_FREQ = 10000000UL;
static constexpr uint8_t  SPI_READ_BIT = 0x80;

IMUHAL::IMUHAL(uint8_t cs_accel, uint8_t cs_gyro, uint8_t spi_port)
    : cs_accel_(cs_accel), cs_gyro_(cs_gyro) {
    switch (spi_port) {
        case 1:  spi_ = &SPI1; break;
        case 2:  spi_ = &SPI2; break;
        default: spi_ = &SPI;  break;
    }
}

// ---------------------------------------------------------------------------
// SPI helpers — Accelerometer
// BMI088 accel requires a dummy byte after the address byte on SPI reads.
// ---------------------------------------------------------------------------

uint8_t IMUHAL::accelRead(uint8_t reg) {
    spi_->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_accel_, LOW);
    spi_->transfer(reg | SPI_READ_BIT);
    spi_->transfer(0x00); // dummy
    uint8_t val = spi_->transfer(0x00);
    digitalWrite(cs_accel_, HIGH);
    spi_->endTransaction();
    return val;
}

void IMUHAL::accelWrite(uint8_t reg, uint8_t data) {
    spi_->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_accel_, LOW);
    spi_->transfer(reg & ~SPI_READ_BIT);
    spi_->transfer(data);
    digitalWrite(cs_accel_, HIGH);
    spi_->endTransaction();
}

void IMUHAL::accelReadBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
    spi_->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_accel_, LOW);
    spi_->transfer(reg | SPI_READ_BIT);
    spi_->transfer(0x00); // dummy
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_->transfer(0x00);
    }
    digitalWrite(cs_accel_, HIGH);
    spi_->endTransaction();
}

// ---------------------------------------------------------------------------
// SPI helpers — Gyroscope
// BMI088 gyro does NOT need a dummy byte.
// ---------------------------------------------------------------------------

uint8_t IMUHAL::gyroRead(uint8_t reg) {
    spi_->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_gyro_, LOW);
    spi_->transfer(reg | SPI_READ_BIT);
    uint8_t val = spi_->transfer(0x00);
    digitalWrite(cs_gyro_, HIGH);
    spi_->endTransaction();
    return val;
}

void IMUHAL::gyroWrite(uint8_t reg, uint8_t data) {
    spi_->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_gyro_, LOW);
    spi_->transfer(reg & ~SPI_READ_BIT);
    spi_->transfer(data);
    digitalWrite(cs_gyro_, HIGH);
    spi_->endTransaction();
}

void IMUHAL::gyroReadBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
    spi_->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_gyro_, LOW);
    spi_->transfer(reg | SPI_READ_BIT);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_->transfer(0x00);
    }
    digitalWrite(cs_gyro_, HIGH);
    spi_->endTransaction();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool IMUHAL::init() {
    pinMode(cs_accel_, OUTPUT);
    pinMode(cs_gyro_,  OUTPUT);
    digitalWrite(cs_accel_, HIGH);
    digitalWrite(cs_gyro_,  HIGH);

    spi_->begin();

    // --- Accelerometer init ---
    accelWrite(ACC_SOFTRESET, 0xB6);
    delay(50);

    // Dummy read to bring SPI interface into defined state after reset
    accelRead(ACC_CHIP_ID);

    if (accelRead(ACC_CHIP_ID) != ACC_CHIP_ID_VAL) {
        return false;
    }

    accelWrite(ACC_PWR_CTRL, 0x04);   // enable accelerometer
    delay(5);
    accelWrite(ACC_PWR_CONF, 0x00);   // active mode
    delay(1);
    accelWrite(ACC_CONF,  ACC_CONF_VAL);
    accelWrite(ACC_RANGE, ACC_RANGE_3G);

    // --- Gyroscope init ---
    gyroWrite(GYR_SOFTRESET, 0xB6);
    delay(50);

    if (gyroRead(GYR_CHIP_ID) != GYR_CHIP_ID_VAL) {
        return false;
    }

    gyroWrite(GYR_LPM1,      0x00);           // normal power mode
    gyroWrite(GYR_RANGE,     GYR_RANGE_500DPS);
    gyroWrite(GYR_BANDWIDTH, GYR_BW_VAL);

    initialized_ = true;
    return true;
}

bool IMUHAL::read(IMUReading& reading) {
    if (!initialized_) return false;

    uint8_t raw[6];

    // Accelerometer
    accelReadBytes(ACC_DATA_START, raw, 6);
    int16_t ax = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    int16_t ay = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    int16_t az = static_cast<int16_t>((raw[5] << 8) | raw[4]);
    reading.accel_x = static_cast<float>(ax) * ACCEL_SCALE;
    reading.accel_y = static_cast<float>(ay) * ACCEL_SCALE;
    reading.accel_z = static_cast<float>(az) * ACCEL_SCALE;

    // Gyroscope
    gyroReadBytes(GYR_DATA_START, raw, 6);
    int16_t gx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    int16_t gy = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    int16_t gz = static_cast<int16_t>((raw[5] << 8) | raw[4]);
    reading.gyro_x = static_cast<float>(gx) * GYRO_SCALE;
    reading.gyro_y = static_cast<float>(gy) * GYRO_SCALE;
    reading.gyro_z = static_cast<float>(gz) * GYRO_SCALE;

    reading.timestamp = millis();
    reading.valid = true;
    return true;
}

void IMUHAL::calibrate(uint16_t samples) {
    if (!initialized_) return;
    (void)samples;
}

bool IMUHAL::selfTest() {
    if (!initialized_) return false;
    return (accelRead(ACC_CHIP_ID) == ACC_CHIP_ID_VAL) &&
           (gyroRead(GYR_CHIP_ID)  == GYR_CHIP_ID_VAL);
}
