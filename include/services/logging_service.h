#pragma once

#include <Arduino.h>
#include <cstdint>
#include "services/data_collection_service.h"
#include "drivers/storage_driver.h"

/**
 * @brief Daten-Logging-Service
 * Speichert alle Telemetrie-Daten auf SD-Karte
 * Format: CSV für einfache Analyse
 */

class LoggingService {
public:
    LoggingService();
    ~LoggingService();
    
    /**
     * @brief Initialisiert SD-Karte und erstellt Log-Dateien
     * @param mission_name Name für Log-Dateien (z.B. "MAGGIE_001")
     * @return true wenn erfolgreich
     */
    bool init(const char* mission_name);
    
    /**
     * @brief Schreibt Telemetrie-Datenpunkt in Log
     * @param data Telemetry-Daten
     * @return true wenn erfolgreich
     */
    bool logTelemetry(const TelemetryData& data);
    
    /**
     * @brief Schreibt Text-Log-Nachricht
     * @param level Log-Level (INFO, WARNING, ERROR)
     * @param message Nachricht
     */
    void logMessage(const char* level, const char* message);
    
    /**
     * @brief Speichert alle ausstehenden Daten
     */
    void flush();
    
    /**
     * @brief Gibt Anzahl gespeicherter Datenpunkte zurück
     */
    uint32_t getLoggedCount() const;
    
    /**
     * @brief Prüft Storage-Zustand
     */
    bool isHealthy() const;

private:
    StorageDriver storage;
    char mission_filename[64];
    char log_filename[64];
    uint32_t logged_count = 0;
    
    static constexpr const char* CSV_HEADER = 
        "timestamp,sequence,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
        "mag_x,mag_y,mag_z,pressure,temperature,altitude,sensor_status";
};
