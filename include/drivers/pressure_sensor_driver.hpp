#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Drucksensor Treiber (SPI)
 * Misst Druck (z.B. für Flüssigkeitsbehälter, Pneumatik)
 * 
 * Unterstützte Sensoren:
 * - MS5607 (SPI/I2C)
 * - BMP390 (I2C/SPI) - bereits vorhanden
 * - Weitere SPI-basierte Drucksensoren
 */

struct PressureReading {
    float pressure;         // Druck in hPa oder bar
    float temperature;      // Temperatur in °C
    float altitude;        // Berechnete Höhe (m)
    uint32_t timestamp;    // Zeitstempel
    bool valid;            // Gültige Messung
};

class PressureSensorDriver {
public:
    /**
     * @brief Konstruktor
     * @param cs_pin Chip Select Pin (SPI)
     */
    PressureSensorDriver(uint8_t cs_pin);
    ~PressureSensorDriver();
    
    /**
     * @brief Initialisiert Drucksensor
     * @param sensor_type Sensor-Typ (z.B. "MS5607", "BMP390")
     * @return true wenn erfolgreich
     */
    bool init(const char* sensor_type = "MS5607");
    
    /**
     * @brief Liest Drucksensor
     * @param reading Zeiger auf PressureReading
     * @return true wenn erfolgreich
     */
    bool read(PressureReading& reading);
    
    /**
     * @brief Setzt Referenzdruck für Höhenberechnung
     * @param ref_pressure Referenzdruck in hPa (z.B. 1013.25 für Meereshöhe)
     */
    void setReferencePressure(float ref_pressure);
    
    /**
     * @brief Gibt aktuellen Druck zurück
     */
    float getPressure() const;
    
    /**
     * @brief Gibt aktuelle Temperatur zurück
     */
    float getTemperature() const;
    
    /**
     * @brief Prüft Sensor-Zustand
     */
    bool isHealthy() const;

private:
    uint8_t cs_pin;
    float reference_pressure = 1013.25f;  // hPa (Meereshöhe)
    
    float last_pressure = 0;
    float last_temperature = 0;
    uint32_t last_read_time = 0;
    
    bool initialized = false;
    
    // SPI-Kommunikation
    uint8_t spiRead(uint8_t reg);
    uint16_t spiRead16(uint8_t reg);
    void spiWrite(uint8_t reg, uint8_t value);
};
