#include "drivers/imu_driver.hpp"
#include <cmath>

// Konversionsfaktoren für BMI088
// Accelerometer: LSB zu m/s²
static constexpr float ACCEL_LSB_3G = 1.0f / 10920.0f;
static constexpr float ACCEL_LSB_6G = 1.0f / 5460.0f;
static constexpr float ACCEL_LSB_12G = 1.0f / 2730.0f;
static constexpr float ACCEL_LSB_24G = 1.0f / 1365.0f;

// Gyroscope: LSB zu rad/s
static constexpr float GYRO_LSB_2000DPS = (2000.0f / 32767.0f) * (3.14159f / 180.0f);  // °/s zu rad/s

IMUDriver::IMUDriver(uint8_t accel_cs_pin, uint8_t gyro_cs_pin)
    : accel_cs_pin(accel_cs_pin), gyro_cs_pin(gyro_cs_pin) {
    // Konstruktor
    pinMode(accel_cs_pin, OUTPUT);
    pinMode(gyro_cs_pin, OUTPUT);
    digitalWrite(accel_cs_pin, HIGH);
    digitalWrite(gyro_cs_pin, HIGH);
}

IMUDriver::~IMUDriver() {
    // Destruktor
}

bool IMUDriver::init() {
    // Starte SPI mit 10 MHz
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2);  // ~20 MHz, wird vom SPI auf max 10 MHz limitiert
    
    delay(100);  // Power-up Delay
    
    // Initialisiere Accelerometer
    if (!initAccelerometer()) {
        Serial.println("ERROR: BMI088 Accelerometer init failed");
        healthy = false;
        return false;
    }
    
    delay(50);
    
    // Initialisiere Gyroscope
    if (!initGyroscope()) {
        Serial.println("ERROR: BMI088 Gyroscope init failed");
        healthy = false;
        return false;
    }
    
    Serial.println("INFO: BMI088 initialized successfully");
    healthy = true;
    return true;
}

bool IMUDriver::initAccelerometer() {
    // Lese Chip-ID
    uint8_t chip_id = spiReadReg(accel_cs_pin, ACCEL_CHIP_ID_REG);
    if (chip_id != ACCEL_CHIP_ID) {
        Serial.printf("ERROR: BMI088 Accel Chip ID mismatch: 0x%02X (expected 0x%02X)\n", chip_id, ACCEL_CHIP_ID);
        return false;
    }
    
    // Power-up Accelerometer (bit 2 und 0 setzen)
    spiWriteReg(accel_cs_pin, ACCEL_POWER_CTRL, 0x04);
    delay(50);
    
    // Range: ±24g (0x03)
    spiWriteReg(accel_cs_pin, ACCEL_RANGE, 0x03);
    accel_range = 3;  // 24g
    
    // Bandwidth: 400 Hz, Sample Rate: 800 Hz (0x0B)
    spiWriteReg(accel_cs_pin, ACCEL_CONF, 0x0B);
    
    Serial.println("INFO: BMI088 Accelerometer initialized");
    return true;
}

bool IMUDriver::initGyroscope() {
    // Lese Chip-ID
    uint8_t chip_id = spiReadReg(gyro_cs_pin, GYRO_CHIP_ID_REG);
    if (chip_id != GYRO_CHIP_ID) {
        Serial.printf("ERROR: BMI088 Gyro Chip ID mismatch: 0x%02X (expected 0x%02X)\n", chip_id, GYRO_CHIP_ID);
        return false;
    }
    
    // Range: ±2000 °/s (0x00)
    spiWriteReg(gyro_cs_pin, GYRO_RANGE, 0x00);
    gyro_range = 0;  // 2000 DPS
    
    // Bandwidth: 230 Hz, Sample Rate: 2000 Hz (0x04)
    spiWriteReg(gyro_cs_pin, GYRO_BANDWIDTH, 0x04);
    
    Serial.println("INFO: BMI088 Gyroscope initialized");
    return true;
}

bool IMUDriver::read(IMUData& data) {
    if (!healthy) return false;
    
    uint8_t buffer[6] = {0};
    int16_t raw_accel[3], raw_gyro[3];
    
    // Lese Accelerometer-Daten (6 Bytes)
    spiReadRegs(accel_cs_pin, ACCEL_DATA_START, buffer, 6);
    raw_accel[0] = ((int16_t)buffer[1] << 8) | buffer[0];
    raw_accel[1] = ((int16_t)buffer[3] << 8) | buffer[2];
    raw_accel[2] = ((int16_t)buffer[5] << 8) | buffer[4];
    
    // Lese Gyroscope-Daten (6 Bytes)
    spiReadRegs(gyro_cs_pin, GYRO_DATA_START, buffer, 6);
    raw_gyro[0] = ((int16_t)buffer[1] << 8) | buffer[0];
    raw_gyro[1] = ((int16_t)buffer[3] << 8) | buffer[2];
    raw_gyro[2] = ((int16_t)buffer[5] << 8) | buffer[4];
    
    // Lese Temperatur (2 Bytes, signed)
    spiReadRegs(gyro_cs_pin, GYRO_TEMP, buffer, 2);
    int16_t raw_temp = ((int16_t)buffer[0] << 8) | buffer[1];
    data.temperature = (float)raw_temp / 512.0f + 23.0f;  // 23°C offset
    
    // Konvertiere Accelerometer
    float accel_scale = ACCEL_LSB_24G;  // 24g range
    data.accel_x = (float)raw_accel[0] * accel_scale - calib.accel_offset_x;
    data.accel_y = (float)raw_accel[1] * accel_scale - calib.accel_offset_y;
    data.accel_z = (float)raw_accel[2] * accel_scale - calib.accel_offset_z;
    
    // Konvertiere Gyroscope
    data.gyro_x = (float)raw_gyro[0] * GYRO_LSB_2000DPS - calib.gyro_bias_x;
    data.gyro_y = (float)raw_gyro[1] * GYRO_LSB_2000DPS - calib.gyro_bias_y;
    data.gyro_z = (float)raw_gyro[2] * GYRO_LSB_2000DPS - calib.gyro_bias_z;
    
    data.timestamp = millis();
    last_read_time = data.timestamp;
    
    return true;
}

uint8_t IMUDriver::spiReadReg(uint8_t cs_pin, uint8_t reg) {
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg | SPI_READ);  // Read-Bit setzen
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    delayMicroseconds(10);
    return value;
}

void IMUDriver::spiWriteReg(uint8_t cs_pin, uint8_t reg, uint8_t value) {
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(cs_pin, HIGH);
    delayMicroseconds(10);
}

void IMUDriver::spiReadRegs(uint8_t cs_pin, uint8_t reg, uint8_t* data, uint8_t length) {
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg | SPI_READ);  // Read-Bit setzen
    for (uint8_t i = 0; i < length; i++) {
        data[i] = SPI.transfer(0x00);
    }
    digitalWrite(cs_pin, HIGH);
    delayMicroseconds(10);
}

void IMUDriver::calibrateAccel() {
    if (!healthy) return;
    
    Serial.println("INFO: Accelerometer calibration started (hold sensor level)");
    
    const uint16_t samples = 1000;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    IMUData data;
    
    for (uint16_t i = 0; i < samples; i++) {
        if (read(data)) {
            sum_x += data.accel_x;
            sum_y += data.accel_y;
            sum_z += data.accel_z - 9.81f;  // Gravitation subtrahieren
        }
        delay(1);
    }
    
    calib.accel_offset_x = sum_x / samples;
    calib.accel_offset_y = sum_y / samples;
    calib.accel_offset_z = sum_z / samples;
    
    Serial.printf("INFO: Accel calibration complete - X:%.4f Y:%.4f Z:%.4f\n",
                 calib.accel_offset_x, calib.accel_offset_y, calib.accel_offset_z);
}

void IMUDriver::calibrateGyro() {
    if (!healthy) return;
    
    Serial.println("INFO: Gyroscope calibration started (keep sensor stationary)");
    
    const uint16_t samples = 1000;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    IMUData data;
    
    for (uint16_t i = 0; i < samples; i++) {
        if (read(data)) {
            sum_x += data.gyro_x;
            sum_y += data.gyro_y;
            sum_z += data.gyro_z;
        }
        delay(1);
    }
    
    calib.gyro_bias_x = sum_x / samples;
    calib.gyro_bias_y = sum_y / samples;
    calib.gyro_bias_z = sum_z / samples;
    
    Serial.printf("INFO: Gyro calibration complete - X:%.6f Y:%.6f Z:%.6f rad/s\n",
                 calib.gyro_bias_x, calib.gyro_bias_y, calib.gyro_bias_z);
}

void IMUDriver::setSampleRate(uint16_t rate) {
    if (!healthy) return;
    
    // BMI088 Accel: 12.5, 25, 50, 100, 200, 400, 800, 1600 Hz
    // Configuration Register für verschiedene Raten
    uint8_t config_value = 0x0B;  // Default: 800 Hz
    
    switch (rate) {
        case 12:  config_value = 0x05; break;
        case 25:  config_value = 0x06; break;
        case 50:  config_value = 0x07; break;
        case 100: config_value = 0x08; break;
        case 200: config_value = 0x09; break;
        case 400: config_value = 0x0A; break;
        case 800: config_value = 0x0B; break;
        case 1600: config_value = 0x0C; break;
        default: return;
    }
    
    spiWriteReg(accel_cs_pin, ACCEL_CONF, config_value);
    Serial.printf("INFO: IMU sample rate set to %d Hz\n", rate);
}

bool IMUDriver::isHealthy() const {
    return healthy;
}
