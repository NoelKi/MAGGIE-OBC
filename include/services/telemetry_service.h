#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstring>

/**
 * @brief Telemetrie-Übertragungsservice
 * Versendet gesammelte Daten über verschiedene Schnittstellen
 * - LoRa (Funk)
 * - CAN Bus
 * - Serial (Debug)
 */

enum class TelemetryChannel {
    SERIAL_DEBUG,    // USB/UART für Debugging
    CAN_BUS,         // CAN-Bus für Avionik
    LORA,            // Funk (Optionale Notfall-Telemetrie)
};

class TelemetryService {
public:
    TelemetryService();
    ~TelemetryService();
    
    /**
     * @brief Initialisiert Telemetrie-Kanäle
     * @return true wenn mindestens ein Kanal initialisiert
     */
    bool init();
    
    /**
     * @brief Versendet Daten als formatierte Zeichenkette
     * @param channel Zielkanal
     * @param data Formatierte Daten-String
     * @return true wenn erfolgreich
     */
    bool transmit(TelemetryChannel channel, const char* data);
    
    /**
     * @brief Versendet Binär-Daten
     * @param channel Zielkanal
     * @param data Zeiger auf Daten
     * @param length Länge in Bytes
     * @return true wenn erfolgreich
     */
    bool transmitBinary(TelemetryChannel channel, const void* data, uint16_t length);
    
    /**
     * @brief Formatiert Telemetrie-Daten als CSV-String
     * @param buffer Ausgabe-Buffer
     * @param buffer_size Größe des Buffers
     * @param sequence Sequenznummer
     * @param accel_x, accel_y, accel_z Beschleunigung
     * @param gyro_x, gyro_y, gyro_z Drehrate
     * @param pressure Luftdruck
     * @param altitude Höhe
     * @param temperature Temperatur
     * @return Länge der formatierten Zeichenkette
     */
    uint16_t formatTelemetryCSV(char* buffer, uint16_t buffer_size,
                                uint32_t sequence,
                                float accel_x, float accel_y, float accel_z,
                                float gyro_x, float gyro_y, float gyro_z,
                                float pressure, float altitude, float temperature);
    
    /**
     * @brief Prüft CAN-Bus-Verbindung
     */
    bool isCANHealthy() const;
    
    /**
     * @brief Prüft Serial-Verbindung
     */
    bool isSerialHealthy() const;

private:
    bool can_initialized = false;
    bool serial_initialized = false;
    bool lora_initialized = false;
    
    void initCAN();
    void initSerial();
    void initLoRa();
};
