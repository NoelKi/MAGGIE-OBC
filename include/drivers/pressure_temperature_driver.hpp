#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Barometer/Thermometer Driver
 * Für Druck- und Temperaturmessung
 * z.B. BMP390, BMP280, MS5607
 */

struct BarometerData {
    float pressure;           // hPa (Hectopascals)
    float temperature;        // °C
    float altitude;          // m (berechnet aus Druck)
    uint32_t timestamp;      // ms
};

class PressureTemperatureDriver {
public:
    PressureTemperatureDriver();
    ~PressureTemperatureDriver();
    
    /**
     * @brief Initialisiert den Barometer-Sensor
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Liest Druck und Temperatur
     * @param data Zeiger auf BarometerData
     * @return true wenn erfolgreich
     */
    bool read(BarometerData& data);
    
    /**
     * @brief Setzt den Referenzdruck für Höhenberechnung
     * @param pressure_hpa Luftdruck auf Meereshöhe (1013.25 hPa)
     */
    void setSeaLevelPressure(float pressure_hpa);
    
    /**
     * @brief Berechnet Höhe aus Druck
     * @param pressure aktuelle Druck in hPa
     * @return Höhe in Metern
     */
    float calculateAltitude(float pressure);
    
    /**
     * @brief Prüft Sensorzustand
     */
    bool isHealthy() const;

private:
    static constexpr uint8_t I2C_ADDRESS = 0x77;  // BMP390/280 Default
    float sea_level_pressure = 1013.25f;  // hPa
    bool healthy = false;
};
