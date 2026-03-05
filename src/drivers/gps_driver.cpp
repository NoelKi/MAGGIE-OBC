#include "drivers/gps_driver.h"

GPSDriver::GPSDriver(uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate)
    : rx_pin(rx_pin), tx_pin(tx_pin), baud_rate(baud_rate) {
    // Konstruktor
}

GPSDriver::~GPSDriver() {
    // Destruktor
}

bool GPSDriver::init() {
    // TODO: Initialisiere Hardware-UART oder SoftwareSerial
    // - Konfiguriere Serial-Port mit baud_rate
    // - Sende Initialisierungsbefehle an GPS-Empfänger
    // - Setze Update-Rate (z.B. 5 Hz)
    
    healthy = true;
    return true;
}

bool GPSDriver::read(GPSData& data) {
    if (!healthy) return false;
    
    if (current_data.valid) {
        data = current_data;
        current_data.valid = false;  // Markiere als gelesen
        return true;
    }
    
    return false;
}

void GPSDriver::update() {
    if (!healthy) return;
    
    // TODO: Lese Daten aus Serial-Buffer
    // - Verarbeite empfangene NMEA-Sätze
    // - Rufe parseNMEA auf
}

void GPSDriver::parseNMEA(const char* sentence) {
    // TODO: Implementiere NMEA-Parser
    // Beispiele:
    // - $GPRMC: Position, Geschwindigkeit, Datum, Zeit
    // - $GPGGA: Position, Höhe, Anzahl Satelliten
    // - $GPGSA: DOP Werte, Satelliten-IDs
}

bool GPSDriver::isHealthy() const {
    return healthy;
}
