#pragma once
#include <Arduino.h>
#include <Wire.h>

/**
 * HAL I2C Interface - Abstrahiert I2C/Wire Kommunikation
 * 
 * Geräte an I2C:
 * - IMU (MPU9250/ICM-20948) auf Addresse 0x68 oder 0x69
 * - Barometer (BMP390/BMP280) auf Addresse 0x76 oder 0x77
 */

namespace HAL {

/**
 * Initialisiert I2C Bus
 * @return true wenn erfolgreich, false bei Fehler
 */
bool i2cInit();

/**
 * Schreibt Daten auf I2C Bus
 * @param address 7-bit I2C Addresse
 * @param data Daten zum Schreiben
 * @param len Länge der Daten
 * @return true wenn erfolgreich, false bei Fehler
 */
bool i2cWrite(uint8_t address, const uint8_t* data, uint8_t len);

/**
 * Liest Daten von I2C Bus
 * @param address 7-bit I2C Addresse
 * @param data Buffer für empfangene Daten
 * @param len Anzahl der zu lesenden Bytes
 * @return true wenn erfolgreich, false bei Fehler
 */
bool i2cRead(uint8_t address, uint8_t* data, uint8_t len);

/**
 * Schreibt ein Register und liest das Ergebnis (Write-Read)
 * @param address 7-bit I2C Addresse
 * @param reg Register zu schreiben
 * @param data Buffer für Daten
 * @param len Länge der zu lesenden Daten
 * @return true wenn erfolgreich, false bei Fehler
 */
bool i2cWriteRead(uint8_t address, uint8_t reg, uint8_t* data, uint8_t len);

/**
 * Scannen und Listen aller I2C Geräte
 * @return Anzahl der gefundenen Geräte
 */
uint8_t i2cScan();

} // namespace HAL
