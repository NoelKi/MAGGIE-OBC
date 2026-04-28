#include "drivers/weight_sensor_driver.hpp"

// ===========================================================================
// Konstruktor – Analog ADC
// ===========================================================================
WeightSensorDriver::WeightSensorDriver(uint8_t pin_scale1, uint8_t pin_scale2)
    : pin_scale1(pin_scale1), pin_scale2(pin_scale2),
      scale_type(ScaleType::ANALOG_ADC) {}

// ===========================================================================
// Konstruktor – HX711 (bogde/HX711-Bibliothek)
// ===========================================================================
WeightSensorDriver::WeightSensorDriver(uint8_t pin_dout1, uint8_t pin_sck1,
                                       uint8_t pin_dout2, uint8_t pin_sck2)
    : pin_dout1_(pin_dout1), pin_sck1_(pin_sck1),
      pin_dout2_(pin_dout2), pin_sck2_(pin_sck2),
      scale_type(ScaleType::HX711) {
    hx711_1_ = new HX711();
    hx711_2_ = new HX711();
}

// ===========================================================================
// Destruktor
// ===========================================================================
WeightSensorDriver::~WeightSensorDriver() {
    delete hx711_1_;
    delete hx711_2_;
}

// ===========================================================================
// init
// ===========================================================================
bool WeightSensorDriver::init(ScaleType type) {
    this->scale_type = type;

    switch (scale_type) {
        // -------------------------------------------------------------------
        case ScaleType::ANALOG_ADC:
            pinMode(pin_scale1, INPUT);
            pinMode(pin_scale2, INPUT);
            Serial.println("INFO  [WeightSensorDriver]: Analog-ADC initialisiert.");
            break;

        // -------------------------------------------------------------------
        case ScaleType::HX711:
            if (!hx711_1_) {
                Serial.println("ERROR [WeightSensorDriver]: HX711-Objekt nicht vorhanden.");
                return false;
            }

            // begin() konfiguriert Pins und Gain (128 = Kanal A, Standard)
            hx711_1_->begin(pin_dout1_, pin_sck1_, 128);
            if (!hx711_1_->wait_ready_timeout(2000)) {
                Serial.println("ERROR [WeightSensorDriver]: HX711 antwortet nicht (Timeout).");
                return false;
            }
            hx711_1_->set_scale(calibration_factor1);

            // Zweiten Sensor nur initialisieren wenn andere Pins verwendet werden
            if (hx711_2_ && (pin_dout2_ != pin_dout1_ || pin_sck2_ != pin_sck1_)) {
                hx711_2_->begin(pin_dout2_, pin_sck2_, 128);
                if (!hx711_2_->wait_ready_timeout(2000)) {
                    Serial.println("WARN  [WeightSensorDriver]: HX711 #2 antwortet nicht – wird ignoriert.");
                    delete hx711_2_;
                    hx711_2_ = nullptr;
                } else {
                    hx711_2_->set_scale(calibration_factor2);
                }
            }

            Serial.println("INFO  [WeightSensorDriver]: HX711 initialisiert.");
            break;

        // -------------------------------------------------------------------
        case ScaleType::I2C_SCALE:
            Serial.println("WARN  [WeightSensorDriver]: I2C-Waage noch nicht implementiert.");
            break;
    }

    initialized = true;
    return true;
}

// ===========================================================================
// readScale1
// ===========================================================================
bool WeightSensorDriver::readScale1(ScaleReading& reading) {
    if (!initialized) return false;

    if (scale_type == ScaleType::HX711) {
        if (!hx711_1_ || !hx711_1_->is_ready()) return false;

        // Einen einzigen Read – alles aus diesem Wert ableiten
        const long raw_full = hx711_1_->read();   // 24-Bit, absolut (kein Tare, kein Scale)
        const long offset   = hx711_1_->get_offset();
        const float scale   = hx711_1_->get_scale();

        reading.raw_value = static_cast<uint32_t>(raw_full - offset);  // Tare-bereinigt
        reading.weight    = (scale != 0.0f)
                                ? static_cast<float>(raw_full - offset) / scale
                                : 0.0f;
        reading.voltage   = 0.0f;
    } else {
        int32_t raw = 0;
        if (!readRawValue(1, raw)) return false;
        reading.raw_value = static_cast<uint32_t>(raw);
        reading.weight    = static_cast<float>(raw) * calibration_factor1;
        reading.voltage   = raw * 3.3f / 1023.0f;
    }

    reading.timestamp = millis();
    reading.valid     = true;
    last_weight1      = reading.weight;
    last_read_time    = reading.timestamp;
    return true;
}

// ===========================================================================
// readScale2
// ===========================================================================
bool WeightSensorDriver::readScale2(ScaleReading& reading) {
    if (!initialized) return false;

    if (scale_type == ScaleType::HX711) {
        if (!hx711_2_ || !hx711_2_->is_ready()) return false;

        const long raw_full = hx711_2_->read();
        const long offset   = hx711_2_->get_offset();
        const float scale   = hx711_2_->get_scale();

        reading.raw_value = static_cast<uint32_t>(raw_full - offset);
        reading.weight    = (scale != 0.0f)
                                ? static_cast<float>(raw_full - offset) / scale
                                : 0.0f;
        reading.voltage   = 0.0f;
    } else {
        int32_t raw = 0;
        if (!readRawValue(2, raw)) return false;
        reading.raw_value = static_cast<uint32_t>(raw);
        reading.weight    = static_cast<float>(raw) * calibration_factor2;
        reading.voltage   = raw * 3.3f / 1023.0f;
    }

    reading.timestamp = millis();
    reading.valid     = true;
    last_weight2      = reading.weight;
    last_read_time    = reading.timestamp;
    return true;
}

// ===========================================================================
// readBothScales
// ===========================================================================
bool WeightSensorDriver::readBothScales(ScaleReading& reading1, ScaleReading& reading2) {
    return readScale1(reading1) && readScale2(reading2);
}

// ===========================================================================
// tareScale1 – nutzt tare() der bogde-Bibliothek (Mittelwert aus 10 Messungen)
// ===========================================================================
void WeightSensorDriver::tareScale1() {
    if (scale_type == ScaleType::HX711 && hx711_1_) {
        hx711_1_->tare(10);
        Serial.printf("INFO  [WeightSensorDriver]: Scale 1 tariert (Offset: %ld)\n",
                      hx711_1_->get_offset());
    } else {
        int32_t raw = 0;
        readRawValue(1, raw);
        Serial.printf("INFO  [WeightSensorDriver]: Scale 1 tariert (Offset: %ld)\n",
                      static_cast<long>(raw));
    }
}

// ===========================================================================
// tareScale2
// ===========================================================================
void WeightSensorDriver::tareScale2() {
    if (scale_type == ScaleType::HX711 && hx711_2_) {
        hx711_2_->tare(10);
        Serial.printf("INFO  [WeightSensorDriver]: Scale 2 tariert (Offset: %ld)\n",
                      hx711_2_->get_offset());
    } else {
        int32_t raw = 0;
        readRawValue(2, raw);
        Serial.printf("INFO  [WeightSensorDriver]: Scale 2 tariert (Offset: %ld)\n",
                      static_cast<long>(raw));
    }
}

// ===========================================================================
// setCalibrationFactor – setzt set_scale() der bogde-Bibliothek
// ===========================================================================
void WeightSensorDriver::setCalibrationFactor(uint8_t scale_id, float factor) {
    if (scale_id == 1) {
        calibration_factor1 = factor;
        if (hx711_1_) hx711_1_->set_scale(factor);
        Serial.printf("INFO  [WeightSensorDriver]: Scale 1 Kalibrierfaktor: %.6f\n", factor);
    } else if (scale_id == 2) {
        calibration_factor2 = factor;
        if (hx711_2_) hx711_2_->set_scale(factor);
        Serial.printf("INFO  [WeightSensorDriver]: Scale 2 Kalibrierfaktor: %.6f\n", factor);
    }
}

// ===========================================================================
// getTotalWeight
// ===========================================================================
float WeightSensorDriver::getTotalWeight() const {
    return last_weight1 + last_weight2;
}

// ===========================================================================
// isHealthy
// ===========================================================================
bool WeightSensorDriver::isHealthy() const {
    return initialized && (millis() - last_read_time) < 5000;
}

// ===========================================================================
// private: readRawValue
// ===========================================================================
bool WeightSensorDriver::readRawValue(uint8_t scale_id, int32_t& raw) {
    switch (scale_type) {
        // -------------------------------------------------------------------
        case ScaleType::ANALOG_ADC: {
            uint8_t pin = (scale_id == 1) ? pin_scale1 : pin_scale2;
            uint32_t sum = 0;
            for (uint8_t i = 0; i < 10; i++) {
                sum += static_cast<uint32_t>(analogRead(pin));
            }
            raw = static_cast<int32_t>(sum / 10);
            return true;
        }

        // -------------------------------------------------------------------
        case ScaleType::HX711: {
            HX711* hx = (scale_id == 1) ? hx711_1_ : hx711_2_;
            if (!hx || !hx->is_ready()) return false;
            raw = static_cast<int32_t>(hx->read());
            return true;
        }

        // -------------------------------------------------------------------
        default:
            return false;
    }
}
