#include <Arduino.h>
#include "hal_adc.hpp"
#include "hal_config.hpp"

namespace HAL {

static ADCResolution currentResolution = ADC_RES_12BIT;
static uint16_t adcCalibrationValues[2] = {0, 0};

bool adcInit(ADCResolution resolution) {
    currentResolution = resolution;
    
    // Konvertiere enum zu uint8_t für analogReadResolution
    uint8_t bits = 12;
    switch (resolution) {
        case ADC_RES_8BIT:
            bits = 8;
            break;
        case ADC_RES_10BIT:
            bits = 10;
            break;
        case ADC_RES_12BIT:
            bits = 12;
            break;
        case ADC_RES_14BIT:
            bits = 14;
            break;
        default:
            bits = 12;
    }
    
    // Setze ADC Auflösung
    analogReadResolution(bits);
    
    // Setze ADC Referenz auf 3.3V (Teensy 4.1 default)
    // analogReference() ist optional - Teensy nutzt standardmäßig 3.3V
    
    // Starte Kalibrierung
    adcCalibrate(HAL_ADC_SCALE1);
    adcCalibrate(HAL_ADC_SCALE2);
    
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "[HAL_ADC] ADC initialized at %d bits", bits);
    Serial.println(buffer);
    
    return true;
}

uint16_t adcRead(uint8_t pin) {
    return analogRead(pin);
}

uint16_t adcReadAverage(uint8_t pin, uint8_t samples) {
    if (samples == 0) samples = 1;
    
    uint32_t sum = 0;
    
    for (uint8_t i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delayMicroseconds(100);  // Kleine Verzögerung zwischen Samples
    }
    
    return sum / samples;
}

float adcToVoltage(uint16_t adcValue) {
    // Teensy 4.1 hat 12-bit ADC mit 3.3V Referenz
    // Spannung = (ADC-Wert / 4095) * 3.3V
    return (adcValue / 4095.0f) * 3.3f;
}

uint16_t adcCalibrate(uint8_t pin) {
    // Lese mehrmals und nehme Durchschnitt
    uint16_t calibValue = adcReadAverage(pin, 20);
    
    // Speichere Kalibrierungswert
    if (pin == HAL_ADC_SCALE1) {
        adcCalibrationValues[0] = calibValue;
    } else if (pin == HAL_ADC_SCALE2) {
        adcCalibrationValues[1] = calibValue;
    }
    
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "[HAL_ADC] Calibrated pin %d with value %u", 
             pin, calibValue);
    Serial.println(buffer);
    
    return calibValue;
}

} // namespace HAL
