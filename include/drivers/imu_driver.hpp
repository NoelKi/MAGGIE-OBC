#pragma once

#include <Arduino.h>
#include <cstdint>
#include <SPI.h>

/**
 * @brief IMU (Inertial Measurement Unit) Driver für BMI088
 * 6-DOF Sensor (Accelerometer + Gyroscope)
 * Kommunikation: SPI (separate Busses für Accel und Gyro)
 * 
 * BMI088 Spezifikationen:
 * - Accel: ±3/6/12/24g
 * - Gyro: ±125/250/500/1000/2000 °/s
 * - SPI-Frequenz: bis 10 MHz
 */

struct IMUData {
    float accel_x, accel_y, accel_z;      // m/s²
    float gyro_x, gyro_y, gyro_z;         // rad/s
    float temperature;                     // °C
    uint32_t timestamp;                   // ms
};

class IMUDriver {
public:
    // Default CS-Pins: Accel auf Pin 10, Gyro auf Pin 9
    IMUDriver(uint8_t accel_cs_pin = 10, uint8_t gyro_cs_pin = 9);
    ~IMUDriver();
    
    /**
     * @brief Initialisiert den BMI088-Sensor über SPI
     * @return true wenn erfolgreich, false bei Fehler
     */
    bool init();
    
    /**
     * @brief Liest aktuelle IMU-Daten (Accel + Gyro + Temp)
     * @param data Zeiger auf IMUData-Struktur
     * @return true wenn erfolgreich
     */
    bool read(IMUData& data);
    
    /**
     * @brief Kalibriert das Accelerometer
     * Sammelt Offsets bei ruhendem Sensor
     */
    void calibrateAccel();
    
    /**
     * @brief Kalibriert das Gyroscope
     * Sammelt Bias bei ruhendem Sensor
     */
    void calibrateGyro();
    
    /**
     * @brief Setzt die Sample-Rate
     * @param rate Hz (12.5, 25, 50, 100, 200, 400, 800, 1600)
     */
    void setSampleRate(uint16_t rate);
    
    /**
     * @brief Prüft ob Sensor verfügbar ist
     */
    bool isHealthy() const;

private:
    // BMI088 Register
    static constexpr uint8_t ACCEL_CHIP_ID_REG = 0x00;
    static constexpr uint8_t ACCEL_CHIP_ID = 0x1A;
    static constexpr uint8_t ACCEL_POWER_CTRL = 0x7D;
    static constexpr uint8_t ACCEL_CONF = 0x40;
    static constexpr uint8_t ACCEL_RANGE = 0x41;
    static constexpr uint8_t ACCEL_DATA_START = 0x12;  // X_LSB
    
    static constexpr uint8_t GYRO_CHIP_ID_REG = 0x00;
    static constexpr uint8_t GYRO_CHIP_ID = 0x0F;
    static constexpr uint8_t GYRO_RANGE = 0x0F;
    static constexpr uint8_t GYRO_BANDWIDTH = 0x10;
    static constexpr uint8_t GYRO_LPM1 = 0x11;
    static constexpr uint8_t GYRO_DATA_START = 0x02;   // X_LSB
    static constexpr uint8_t GYRO_TEMP = 0x21;
    
    static constexpr uint8_t SPI_READ = 0x80;          // Read-Bit für SPI
    
    // Kalibrierungsdaten
    struct CalibData {
        float accel_offset_x = 0.0f;
        float accel_offset_y = 0.0f;
        float accel_offset_z = 0.0f;
        float gyro_bias_x = 0.0f;
        float gyro_bias_y = 0.0f;
        float gyro_bias_z = 0.0f;
    } calib;
    
    uint8_t accel_cs_pin, gyro_cs_pin;
    uint8_t accel_range = 0;              // 0=±3g, 1=±6g, 2=±12g, 3=±24g
    uint8_t gyro_range = 0;               // 0=±2000°/s
    bool healthy = false;
    uint32_t last_read_time = 0;
    
    // SPI-Hilfsfunktionen
    uint8_t spiReadReg(uint8_t cs_pin, uint8_t reg);
    void spiWriteReg(uint8_t cs_pin, uint8_t reg, uint8_t value);
    void spiReadRegs(uint8_t cs_pin, uint8_t reg, uint8_t* data, uint8_t length);
    
    // Sensor-Initialisierung
    bool initAccelerometer();
    bool initGyroscope();
};
