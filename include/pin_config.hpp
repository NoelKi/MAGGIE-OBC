#pragma once

/**
 * @file pin_config.hpp
 * @brief Zentrale Pin-Belegung für den Teensy 4.1 OBC
 *
 * Alle Hardware-Pins an einer Stelle definieren – so muss bei einer
 * Neuverdrahtung nur diese Datei geändert werden.
 */

// ===========================================================================
// Gewichtssensor (HX711 Load-Cell Amplifier)
// ===========================================================================
// Ein Sensor an Pin 2 (DOUT) und Pin 3 (PD_SCK)
static constexpr uint8_t PIN_HX711_DOUT = 2;   ///< DOUT   → Teensy Pin 2 (Digital IN)
static constexpr uint8_t PIN_HX711_SCK  = 3;   ///< PD_SCK → Teensy Pin 3 (Digital OUT)

// ===========================================================================
// Kalibrierung (Startwert – anpassen nach Kalibriervorgang)
// ===========================================================================
// Einheit: Gramm pro HX711-LSB
// Formel:  factor = bekanntes_gewicht_g / (rohwert_mit_last - rohwert_leer)
static constexpr float WEIGHT_CALIBRATION_FACTOR = 1.0f;

// ===========================================================================
// Ausleseintervall
// ===========================================================================
static constexpr uint32_t WEIGHT_READ_INTERVAL_MS = 500;  ///< alle 500 ms auslesen
