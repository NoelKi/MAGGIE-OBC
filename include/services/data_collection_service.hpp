#pragma once

#include <Arduino.h>
#include <cstdint>
#include "drivers/imu_driver.hpp"
#include "drivers/pressure_temperature_driver.hpp"

/**
 * @brief Datenerfassungs-Service
 * Koordiniert alle Sensoren und sammelt Telemetrie-Daten
 */

struct TelemetryData {
    // IMU-Daten
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    
    // Barometer-Daten
    float pressure;
    float temperature;
    float altitude;
    
    // Zusätzliche Informationen
    uint32_t timestamp;
    uint32_t sequence;
    uint8_t sensor_status;
};

class DataCollectionService {
public:
    DataCollectionService();
    ~DataCollectionService();
    
    /**
     * @brief Initialisiert alle Sensoren
     * @return true wenn alle Sensoren erfolgreich initialisiert
     */
    bool init();
    
    /**
     * @brief Liest alle Sensoren und sammelt Daten
     * @param data Zeiger auf TelemetryData
     * @return true wenn erfolgreich
     */
    bool collectData(TelemetryData& data);
    
    /**
     * @brief Gibt Sensor-Status zurück
     */
    uint8_t getSensorStatus() const;
    
    /**
     * @brief Kalibriert alle Sensoren
     */
    void calibrateAll();
    
    /**
     * @brief Setzt Sample-Rate für Sensoren
     * @param rate_hz Frequenz in Hz
     */
    void setSampleRate(uint16_t rate_hz);
    
    /**
     * @brief Gibt Anzahl gesammelter Datenpunkte zurück
     */
    uint32_t getDataPointCount() const;

private:
    IMUDriver imu;
    PressureTemperatureDriver barometer;
    
    uint32_t data_count = 0;
    uint32_t last_sample_time = 0;
    uint16_t target_sample_rate = 100;  // 100 Hz Standard
    
    uint8_t getStatusByte() const;
};
