#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief GPS-Empfänger Driver
 * Für Positionsbestimmung (optionale Funktion)
 * z.B. NEO-6M, NEO-M9N (UART/Serial)
 */

struct GPSData {
    float latitude;           // Dezimalgrad
    float longitude;          // Dezimalgrad
    float altitude;          // m
    float speed;             // m/s
    uint16_t satellites;     // Anzahl verbundener Satelliten
    uint32_t timestamp;      // ms
    bool valid;              // true wenn gültige Daten
};

class GPSDriver {
public:
    GPSDriver(uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate = 9600);
    ~GPSDriver();
    
    /**
     * @brief Initialisiert GPS-Empfänger
     */
    bool init();
    
    /**
     * @brief Liest GPS-Daten (non-blocking)
     * @param data Zeiger auf GPSData
     * @return true wenn neue Daten verfügbar
     */
    bool read(GPSData& data);
    
    /**
     * @brief Verarbeitet eingehende NMEA-Daten
     * Sollte regelmäßig aufgerufen werden
     */
    void update();
    
    /**
     * @brief Prüft Empfänger-Zustand
     */
    bool isHealthy() const;

private:
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint32_t baud_rate;
    bool healthy = false;
    GPSData current_data = {};
    
    void parseNMEA(const char* sentence);
};
