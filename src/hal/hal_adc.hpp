#pragma once
#include <Arduino.h>

/**
 * HAL ADC Interface - Abstrahiert analog zu digital Konvertierung
 * 
 * ADC-Kanäle:
 * - A0 (HAL_ADC_SCALE1): Gewichtssensor 1
 * - A1 (HAL_ADC_SCALE2): Gewichtssensor 2
 */

namespace HAL {

// ADC Auflösungen
enum ADCResolution {
    ADC_RES_8BIT = 8,
    ADC_RES_10BIT = 10,
    ADC_RES_12BIT = 12,
    ADC_RES_14BIT = 14
};

/**
 * Initialisiert ADC
 * @param resolution 8, 10, 12, oder 14 Bits
 * @return true wenn erfolgreich
 */
bool adcInit(ADCResolution resolution = ADC_RES_12BIT);

/**
 * Liest einen ADC-Kanal
 * @param pin ADC Pin (A0, A1, etc.)
 * @return ADC-Wert (0 bis max, abhängig von Auflösung)
 */
uint16_t adcRead(uint8_t pin);

/**
 * Liest mehrmals und gibt Durchschnitt zurück (für Stabilität)
 * @param pin ADC Pin
 * @param samples Anzahl der Samples (default 10)
 * @return Durchschnittswert
 */
uint16_t adcReadAverage(uint8_t pin, uint8_t samples = 10);

/**
 * Konvertiert ADC-Wert zu Spannung (0.0 - 3.3V)
 * @param adcValue ADC-Wert
 * @return Spannung in Volt
 */
float adcToVoltage(uint16_t adcValue);

/**
 * Kallibriert einen ADC-Kanal (Nullpunkt)
 * @param pin ADC Pin zu kallibrieren
 * @return Kalibrierungswert (sollte subtrahiert werden)
 */
uint16_t adcCalibrate(uint8_t pin);

} // namespace HAL
