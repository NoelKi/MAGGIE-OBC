#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief IMU (Inertial Measurement Unit) Driver
 * Unterstützt 9-DOF Sensoren (Accelerometer, Gyroscope, Magnetometer)
 * z.B. MPU9250, ICM-20948
 */

struct IMUData {
    float accel_x, accel_y, accel_z;      // m/s²
    float gyro_x, gyro_y, gyro_z;         // rad/s
    float mag_x, mag_y, mag_z;            // µT
    float temperature;                     // °C
    uint32_t timestamp;                   // ms
};

class IMUDriver {
public:
    IMUDriver();
    ~IMUDriver();
    
    /**
     * @brief Initialisiert den IMU-Sensor
     * @return true wenn erfolgreich, false bei Fehler
     */
    bool init();
    
    /**
     * @brief Liest aktuelle IMU-Daten
     * @param data Zeiger auf IMUData-Struktur
     * @return true wenn erfolgreich
     */
    bool read(IMUData& data);
    
    /**
     * @brief Kalibriert das Accelerometer
     */
    void calibrateAccel();
    
    /**
     * @brief Kalibriert das Gyroscope
     */
    void calibrateGyro();
    
    /**
     * @brief Setzt die Sample-Rate
     * @param rate Hz (1-1000)
     */
    void setSampleRate(uint16_t rate);
    
    /**
     * @brief Prüft ob Sensor verfügbar ist
     */
    bool isHealthy() const;

private:
    static constexpr uint8_t I2C_ADDRESS = 0x68;  // MPU9250 Default
    bool healthy = false;
    uint32_t last_read_time = 0;
};
