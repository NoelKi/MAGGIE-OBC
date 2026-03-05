#include "services/telemetry_service.h"
#include <cstdio>

TelemetryService::TelemetryService() {
    // Konstruktor
}

TelemetryService::~TelemetryService() {
    // Destruktor
}

bool TelemetryService::init() {
    initSerial();
    initCAN();
    initLoRa();  // Optional
    
    return serial_initialized || can_initialized;
}

bool TelemetryService::transmit(TelemetryChannel channel, const char* data) {
    if (!data) return false;
    
    switch (channel) {
        case TelemetryChannel::SERIAL_DEBUG:
            if (serial_initialized) {
                Serial.println(data);
                return true;
            }
            break;
            
        case TelemetryChannel::CAN_BUS:
            if (can_initialized) {
                // TODO: Implementiere CAN-Übertragung
                return true;
            }
            break;
            
        case TelemetryChannel::LORA:
            if (lora_initialized) {
                // TODO: Implementiere LoRa-Übertragung
                return true;
            }
            break;
    }
    
    return false;
}

bool TelemetryService::transmitBinary(TelemetryChannel channel, const void* data, uint16_t length) {
    if (!data || length == 0) return false;
    
    switch (channel) {
        case TelemetryChannel::SERIAL_DEBUG:
            if (serial_initialized) {
                Serial.write((const uint8_t*)data, length);
                return true;
            }
            break;
            
        case TelemetryChannel::CAN_BUS:
            if (can_initialized) {
                // TODO: Implementiere binäre CAN-Übertragung
                return true;
            }
            break;
            
        case TelemetryChannel::LORA:
            if (lora_initialized) {
                // TODO: Implementiere binäre LoRa-Übertragung
                return true;
            }
            break;
    }
    
    return false;
}

uint16_t TelemetryService::formatTelemetryCSV(char* buffer, uint16_t buffer_size,
                                              uint32_t sequence,
                                              float accel_x, float accel_y, float accel_z,
                                              float gyro_x, float gyro_y, float gyro_z,
                                              float pressure, float altitude, float temperature) {
    if (!buffer || buffer_size < 100) return 0;
    
    // Format: seq,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,pressure,altitude,temp
    int len = snprintf(buffer, buffer_size,
                      "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
                      sequence,
                      accel_x, accel_y, accel_z,
                      gyro_x, gyro_y, gyro_z,
                      pressure, altitude, temperature);
    
    return (len > 0) ? len : 0;
}

bool TelemetryService::isCANHealthy() const {
    return can_initialized;
}

bool TelemetryService::isSerialHealthy() const {
    return serial_initialized;
}

void TelemetryService::initSerial() {
    Serial.begin(115200);  // Standard Baudrate
    
    // Warte auf Serial-Verbindung
    uint32_t timeout = millis() + 3000;
    while (!Serial && millis() < timeout) {
        delay(100);
    }
    
    serial_initialized = true;
    Serial.println("INFO: Serial telemetry initialized");
}

void TelemetryService::initCAN() {
    // TODO: Initialisiere CAN-Bus Schnittstellenshell
    // Verwende HAL Layer aus hal_can.h
    
    can_initialized = false;  // Bis implementiert
}

void TelemetryService::initLoRa() {
    // TODO: Initialisiere LoRa-Modul
    // Optionaler Kanal für Notfall-Telemetrie
    
    lora_initialized = false;  // Bis implementiert
}
