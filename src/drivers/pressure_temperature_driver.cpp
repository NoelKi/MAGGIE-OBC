#include "drivers/pressure_temperature_driver.hpp"
#include <Wire.h>
#include <cmath>

PressureTemperatureDriver::PressureTemperatureDriver() {
    // Konstruktor
}

PressureTemperatureDriver::~PressureTemperatureDriver() {
    // Destruktor
}

bool PressureTemperatureDriver::init() {
    Wire.begin();
    Wire.setClock(400000);
    
    delay(100);
    
    // Prüfe Sensor-Verbindung
    Wire.beginTransmission(I2C_ADDRESS);
    if (Wire.endTransmission() == 0) {
        healthy = true;
        // TODO: Spezifische Initialisierung basierend auf Sensor-Typ
        return true;
    }
    
    healthy = false;
    return false;
}

bool PressureTemperatureDriver::read(BarometerData& data) {
    if (!healthy) return false;
    
    // TODO: Implementiere Daten-Leseverfahren für spezifischen Sensor
    // - Lese Rohwerte aus Sensor
    // - Wende Kalibrierungskoeffizienten an
    // - Konvertiere zu hPa und °C
    
    data.altitude = calculateAltitude(data.pressure);
    data.timestamp = millis();
    
    return true;
}

void PressureTemperatureDriver::setSeaLevelPressure(float pressure_hpa) {
    sea_level_pressure = pressure_hpa;
}

float PressureTemperatureDriver::calculateAltitude(float pressure) {
    // Barometrische Höhenformel (International Standard Atmosphere)
    // h = (44330 m) * (1 - (P / P0)^(1/5.255))
    
    if (pressure <= 0) return 0;
    
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 1.0f / 5.255f));
}

bool PressureTemperatureDriver::isHealthy() const {
    return healthy;
}
