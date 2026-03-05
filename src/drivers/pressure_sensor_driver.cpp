#include "drivers/pressure_sensor_driver.h"
#include <SPI.h>
#include <cmath>

PressureSensorDriver::PressureSensorDriver(uint8_t cs_pin)
    : cs_pin(cs_pin) {
    // Konstruktor
}

PressureSensorDriver::~PressureSensorDriver() {
    // Destruktor
}

bool PressureSensorDriver::init(const char* sensor_type) {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);  // CS inaktiv
    
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);  // Langsame SPI-Frequenz
    
    // TODO: Sensor-spezifische Initialisierung
    // Für MS5607:
    // - Reset durchführen
    // - Kalibrierungskoeffizienten lesen
    // - ADC-Auflösung setzen
    
    Serial.printf("INFO: Pressure sensor initialized (%s)\n", sensor_type);
    initialized = true;
    return true;
}

bool PressureSensorDriver::read(PressureReading& reading) {
    if (!initialized) return false;
    
    // TODO: Implementiere Druck-Messung
    // 1. Starte ADC-Umwandlung
    // 2. Warte auf Fertigstellung
    // 3. Lese ADC-Wert
    // 4. Berechne Druck und Temperatur
    // 5. Berechne Höhe
    
    reading.pressure = last_pressure;
    reading.temperature = last_temperature;
    reading.timestamp = millis();
    reading.valid = true;
    reading.altitude = 44330.0f * (1.0f - powf(reading.pressure / reference_pressure, 1.0f / 5.255f));
    
    last_read_time = reading.timestamp;
    
    return true;
}

void PressureSensorDriver::setReferencePressure(float ref_pressure) {
    reference_pressure = ref_pressure;
    Serial.printf("INFO: Reference pressure set to %.2f hPa\n", ref_pressure);
}

float PressureSensorDriver::getPressure() const {
    return last_pressure;
}

float PressureSensorDriver::getTemperature() const {
    return last_temperature;
}

bool PressureSensorDriver::isHealthy() const {
    return initialized && (millis() - last_read_time) < 5000;
}

uint8_t PressureSensorDriver::spiRead(uint8_t reg) {
    digitalWrite(cs_pin, LOW);
    
    // Für SPI Read: Register mit MSB gesetzt
    SPI.transfer(0x80 | reg);
    uint8_t value = SPI.transfer(0x00);
    
    digitalWrite(cs_pin, HIGH);
    return value;
}

uint16_t PressureSensorDriver::spiRead16(uint8_t reg) {
    digitalWrite(cs_pin, LOW);
    
    SPI.transfer(0x80 | reg);
    uint16_t value = (uint16_t)SPI.transfer(0x00) << 8;
    value |= SPI.transfer(0x00);
    
    digitalWrite(cs_pin, HIGH);
    return value;
}

void PressureSensorDriver::spiWrite(uint8_t reg, uint8_t value) {
    digitalWrite(cs_pin, LOW);
    
    SPI.transfer(reg);  // Register ohne MSB
    SPI.transfer(value);
    
    digitalWrite(cs_pin, HIGH);
}
