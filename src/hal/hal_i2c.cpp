#include "hal_i2c.h"
#include "hal_config.h"

namespace HAL {

// I2C Wire Objekt (Teensy nutzt Wire für I2C0)
static TwoWire* i2cBus = &Wire;

bool i2cInit() {
    try {
        // Starte I2C auf definierten Pins
        i2cBus->begin();
        i2cBus->setClock(HAL_I2C_FREQ);
        
        Serial.println("[HAL_I2C] I2C Bus initialized at " + String(HAL_I2C_FREQ) + " Hz");
        return true;
    } catch (...) {
        Serial.println("[HAL_I2C] ERROR: Failed to initialize I2C bus");
        return false;
    }
}

bool i2cWrite(uint8_t address, const uint8_t* data, uint8_t len) {
    if (!data || len == 0) {
        return false;
    }

    i2cBus->beginTransmission(address);
    
    for (uint8_t i = 0; i < len; i++) {
        i2cBus->write(data[i]);
    }
    
    uint8_t error = i2cBus->endTransmission();
    return (error == 0);
}

bool i2cRead(uint8_t address, uint8_t* data, uint8_t len) {
    if (!data || len == 0) {
        return false;
    }

    uint8_t bytesRead = i2cBus->requestFrom((int)address, (int)len);
    
    if (bytesRead != len) {
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        if (i2cBus->available()) {
            data[i] = i2cBus->read();
        } else {
            return false;
        }
    }

    return true;
}

bool i2cWriteRead(uint8_t address, uint8_t reg, uint8_t* data, uint8_t len) {
    // Schreibe Register
    if (!i2cWrite(address, &reg, 1)) {
        return false;
    }

    // Kurze Verzögerung
    delayMicroseconds(100);

    // Lese Daten
    return i2cRead(address, data, len);
}

uint8_t i2cScan() {
    uint8_t count = 0;
    
    Serial.println("[HAL_I2C] Scanning I2C bus...");
    
    for (uint8_t address = 1; address < 127; address++) {
        i2cBus->beginTransmission(address);
        
        if (i2cBus->endTransmission() == 0) {
            Serial.print("  Device found at 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            count++;
        }
    }

    Serial.println("[HAL_I2C] Scan complete. Found " + String(count) + " devices");
    return count;
}

} // namespace HAL
