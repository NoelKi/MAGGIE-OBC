#include "services/logging_service.hpp"
#include <cstdio>
#include <cstring>

LoggingService::LoggingService() {
    // Konstruktor
}

LoggingService::~LoggingService() {
    flush();
}

bool LoggingService::init(const char* mission_name) {
    if (!mission_name) return false;
    
    // Erstelle Dateinamen
    snprintf(mission_filename, sizeof(mission_filename), "/%s_telemetry.csv", mission_name);
    snprintf(log_filename, sizeof(log_filename), "/%s_events.log", mission_name);
    
    if (!storage.init()) {
        Serial.println("ERROR: Storage initialization failed");
        return false;
    }
    
    // Erstelle CSV-Datei mit Header
    if (!storage.createCSVFile(mission_filename, CSV_HEADER)) {
        Serial.println("ERROR: Failed to create CSV file");
        return false;
    }
    
    Serial.printf("INFO: Logging initialized with files: %s, %s\n", mission_filename, log_filename);
    
    return true;
}

bool LoggingService::logTelemetry(const TelemetryData& data) {
    if (!storage.isHealthy()) return false;
    
    char row[512];
    int len = snprintf(row, sizeof(row),
                      "%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%u",
                      data.timestamp,
                      data.sequence,
                      data.accel_x, data.accel_y, data.accel_z,
                      data.gyro_x, data.gyro_y, data.gyro_z,
                      data.mag_x, data.mag_y, data.mag_z,
                      data.pressure, data.temperature, data.altitude,
                      data.sensor_status);
    
    if (len > 0 && len < (int)sizeof(row)) {
        if (storage.appendCSVRow(mission_filename, row)) {
            logged_count++;
            return true;
        }
    }
    
    return false;
}

void LoggingService::logMessage(const char* level, const char* message) {
    if (!level || !message) return;
    
    char log_entry[256];
    snprintf(log_entry, sizeof(log_entry), "[%lu] %s: %s", millis(), level, message);
    
    // TODO: Schreibe in Log-Datei
    Serial.println(log_entry);
}

void LoggingService::flush() {
    // TODO: Implementiere Flush-Operationen
    // - Schreibe SD-Karten-Buffer
    // - Synchronisiere Dateisystem
}

uint32_t LoggingService::getLoggedCount() const {
    return logged_count;
}

bool LoggingService::isHealthy() const {
    return storage.isHealthy();
}
