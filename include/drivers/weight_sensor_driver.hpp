#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Waagen-/Gewichtssensor Treiber
 * Liest zwei Load-Cell / Waagen-Sensoren via ADC
 * 
 * Unterstützt:
 * - Einfache ADC-Eingänge (analog)
 * - HX711 Load-Cell Amplifier (optional)
 * - Eichung und Kalibrierung
 */

enum class ScaleType {
    ANALOG_ADC,     // Direkt über ADC (analog Pins)
    HX711,          // HX711 Load-Cell Amplifier
    I2C_SCALE,      // I2C Waage-Modul
};

struct ScaleReading {
    float weight;           // Gewicht in Gramm
    uint32_t raw_value;     // Rohwert vom ADC
    float voltage;          // Gemessene Spannung
    uint32_t timestamp;     // Zeitstempel
    bool valid;             // Gültige Messung
};

class WeightSensorDriver {
public:
    /**
     * @brief Konstruktor für Analog-Waage
     * @param pin_scale1 Analog-Pin für Waage 1
     * @param pin_scale2 Analog-Pin für Waage 2
     */
    WeightSensorDriver(uint8_t pin_scale1, uint8_t pin_scale2);
    
    /**
     * @brief Konstruktor für HX711-Waage
     * @param pin_dout1 Data Out Pin von HX711 (Waage 1)
     * @param pin_sck1 Serial Clock Pin von HX711 (Waage 1)
     * @param pin_dout2 Data Out Pin von HX711 (Waage 2)
     * @param pin_sck2 Serial Clock Pin von HX711 (Waage 2)
     */
    WeightSensorDriver(uint8_t pin_dout1, uint8_t pin_sck1, 
                      uint8_t pin_dout2, uint8_t pin_sck2);
    
    ~WeightSensorDriver();
    
    /**
     * @brief Initialisiert die Waagen-Sensoren
     * @param scale_type Sensor-Typ
     * @return true wenn erfolgreich
     */
    bool init(ScaleType scale_type = ScaleType::ANALOG_ADC);
    
    /**
     * @brief Liest Gewicht von Waage 1
     * @param reading Zeiger auf ScaleReading
     * @return true wenn erfolgreich
     */
    bool readScale1(ScaleReading& reading);
    
    /**
     * @brief Liest Gewicht von Waage 2
     * @param reading Zeiger auf ScaleReading
     * @return true wenn erfolgreich
     */
    bool readScale2(ScaleReading& reading);
    
    /**
     * @brief Liest beide Waagen
     * @param reading1 Zeiger auf ScaleReading für Waage 1
     * @param reading2 Zeiger auf ScaleReading für Waage 2
     * @return true wenn beide erfolgreich
     */
    bool readBothScales(ScaleReading& reading1, ScaleReading& reading2);
    
    /**
     * @brief Tariert Waage 1 (setzt Nullpunkt)
     */
    void tareScale1();
    
    /**
     * @brief Tariert Waage 2 (setzt Nullpunkt)
     */
    void tareScale2();
    
    /**
     * @brief Setzt Kalibrierungsfaktor
     * @param scale_id 1 oder 2
     * @param factor Kalibrierungsfaktor (g pro ADC-Einheit)
     */
    void setCalibrationFactor(uint8_t scale_id, float factor);
    
    /**
     * @brief Gibt Gesamtgewicht beider Waagen zurück
     */
    float getTotalWeight() const;
    
    /**
     * @brief Prüft Sensor-Zustand
     */
    bool isHealthy() const;

private:
    uint8_t pin_scale1;
    uint8_t pin_scale2;
    uint8_t pin_dout1, pin_sck1;
    uint8_t pin_dout2, pin_sck2;
    
    ScaleType scale_type;
    bool initialized = false;
    
    float calibration_factor1 = 0.001f;  // g pro ADC-Einheit
    float calibration_factor2 = 0.001f;
    uint32_t tare_offset1 = 0;
    uint32_t tare_offset2 = 0;
    
    float last_weight1 = 0;
    float last_weight2 = 0;
    uint32_t last_read_time = 0;
    
    uint32_t readRawValue(uint8_t scale_id);
};
