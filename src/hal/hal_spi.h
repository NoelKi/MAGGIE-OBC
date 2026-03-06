#pragma once
#include <Arduino.h>
#include <SPI.h>

/**
 * HAL SPI Interface - Abstrahiert SPI Kommunikation
 * 
 * Geräte an SPI:
 * - Drucksensor (MS5607) auf Pin 10 (CS)
 * - SD-Card Module auf Pin 4 (CS)
 */

namespace HAL {

// SPI Transfer Speed
enum SPISpeed {
    SPI_SPEED_1MHZ = 1000000,
    SPI_SPEED_5MHZ = 5000000,
    SPI_SPEED_10MHZ = 10000000,
    SPI_SPEED_20MHZ = 20000000
};

/**
 * Initialisiert SPI Bus
 * @return true wenn erfolgreich, false bei Fehler
 */
bool spiInit();

/**
 * Schreibt und liest Daten vom SPI (vollduplex)
 * @param csPin Chip Select Pin
 * @param data Zeiger auf Daten (Input/Output)
 * @param len Länge der zu transferierenden Bytes
 * @return true wenn erfolgreich, false bei Fehler
 */
bool spiTransfer(uint8_t csPin, uint8_t* data, uint8_t len);

/**
 * Schreibt Daten auf SPI
 * @param csPin Chip Select Pin
 * @param data Daten zum Schreiben
 * @param len Länge der Daten
 * @return true wenn erfolgreich, false bei Fehler
 */
bool spiWrite(uint8_t csPin, const uint8_t* data, uint8_t len);

/**
 * Liest Daten von SPI
 * @param csPin Chip Select Pin
 * @param data Buffer für empfangene Daten
 * @param len Anzahl der zu lesenden Bytes
 * @return true wenn erfolgreich, false bei Fehler
 */
bool spiRead(uint8_t csPin, uint8_t* data, uint8_t len);

/**
 * Liest ein Register und dessen Wert
 * @param csPin Chip Select Pin
 * @param reg Register zu lesen
 * @param value Zeiger auf Wert
 * @return true wenn erfolgreich, false bei Fehler
 */
bool spiReadRegister(uint8_t csPin, uint8_t reg, uint8_t& value);

/**
 * Schreibt einen Registerwert
 * @param csPin Chip Select Pin
 * @param reg Register zu schreiben
 * @param value Wert
 * @return true wenn erfolgreich, false bei Fehler
 */
bool spiWriteRegister(uint8_t csPin, uint8_t reg, uint8_t value);

} // namespace HAL
