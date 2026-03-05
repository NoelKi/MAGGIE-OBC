#include "drivers/weight_sensor_driver.h"

WeightSensorDriver::WeightSensorDriver(uint8_t pin_scale1, uint8_t pin_scale2)
    : pin_scale1(pin_scale1), pin_scale2(pin_scale2), scale_type(ScaleType::ANALOG_ADC) {
    // Konstruktor für Analog-Sensoren
}

WeightSensorDriver::WeightSensorDriver(uint8_t pin_dout1, uint8_t pin_sck1,
                                       uint8_t pin_dout2, uint8_t pin_sck2)
    : pin_dout1(pin_dout1), pin_sck1(pin_sck1),
      pin_dout2(pin_dout2), pin_sck2(pin_sck2), 
      scale_type(ScaleType::HX711) {
    // Konstruktor für HX711-Sensoren
}

WeightSensorDriver::~WeightSensorDriver() {
    // Destruktor
}

bool WeightSensorDriver::init(ScaleType scale_type) {
    this->scale_type = scale_type;
    
    switch (scale_type) {
        case ScaleType::ANALOG_ADC:
            pinMode(pin_scale1, INPUT);
            pinMode(pin_scale2, INPUT);
            Serial.println("INFO: Weight sensors initialized (Analog ADC)");
            break;
            
        case ScaleType::HX711:
            // TODO: Initialisiere HX711 Sensoren
            // pinMode(pin_dout1, INPUT);
            // pinMode(pin_sck1, OUTPUT);
            // pinMode(pin_dout2, INPUT);
            // pinMode(pin_sck2, OUTPUT);
            Serial.println("INFO: Weight sensors initialized (HX711)");
            break;
            
        case ScaleType::I2C_SCALE:
            // TODO: Initialisiere I2C Waagen-Module
            Serial.println("INFO: Weight sensors initialized (I2C)");
            break;
    }
    
    initialized = true;
    return true;
}

bool WeightSensorDriver::readScale1(ScaleReading& reading) {
    if (!initialized) return false;
    
    uint32_t raw = readRawValue(1);
    reading.raw_value = raw;
    reading.timestamp = millis();
    reading.weight = (raw - tare_offset1) * calibration_factor1;
    reading.valid = true;
    
    last_weight1 = reading.weight;
    last_read_time = reading.timestamp;
    
    return true;
}

bool WeightSensorDriver::readScale2(ScaleReading& reading) {
    if (!initialized) return false;
    
    uint32_t raw = readRawValue(2);
    reading.raw_value = raw;
    reading.timestamp = millis();
    reading.weight = (raw - tare_offset2) * calibration_factor2;
    reading.valid = true;
    
    last_weight2 = reading.weight;
    last_read_time = reading.timestamp;
    
    return true;
}

bool WeightSensorDriver::readBothScales(ScaleReading& reading1, ScaleReading& reading2) {
    return readScale1(reading1) && readScale2(reading2);
}

void WeightSensorDriver::tareScale1() {
    tare_offset1 = readRawValue(1);
    Serial.printf("INFO: Scale 1 tared (offset: %lu)\n", tare_offset1);
}

void WeightSensorDriver::tareScale2() {
    tare_offset2 = readRawValue(2);
    Serial.printf("INFO: Scale 2 tared (offset: %lu)\n", tare_offset2);
}

void WeightSensorDriver::setCalibrationFactor(uint8_t scale_id, float factor) {
    if (scale_id == 1) {
        calibration_factor1 = factor;
        Serial.printf("INFO: Scale 1 calibration factor set to %.6f\n", factor);
    } else if (scale_id == 2) {
        calibration_factor2 = factor;
        Serial.printf("INFO: Scale 2 calibration factor set to %.6f\n", factor);
    }
}

float WeightSensorDriver::getTotalWeight() const {
    return last_weight1 + last_weight2;
}

bool WeightSensorDriver::isHealthy() const {
    return initialized && (millis() - last_read_time) < 5000;
}

uint32_t WeightSensorDriver::readRawValue(uint8_t scale_id) {
    switch (scale_type) {
        case ScaleType::ANALOG_ADC: {
            uint8_t pin = (scale_id == 1) ? pin_scale1 : pin_scale2;
            // Mehrfache Lesevorgänge für Durchschnittswert
            uint32_t sum = 0;
            for (int i = 0; i < 10; i++) {
                sum += analogRead(pin);
            }
            return sum / 10;  // Durchschnittswert
        }
        
        case ScaleType::HX711: {
            // TODO: Implementiere HX711 Leseverfahren
            // uint8_t dout_pin = (scale_id == 1) ? pin_dout1 : pin_dout2;
            // uint8_t sck_pin = (scale_id == 1) ? pin_sck1 : pin_sck2;
            // Lese 24-Bit Wert von HX711
            return 0;  // Placeholder
        }
        
        default:
            return 0;
    }
}
