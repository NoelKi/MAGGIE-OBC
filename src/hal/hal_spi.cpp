#include "hal_spi.h"
#include "hal_config.h"

namespace HAL {

bool spiInit() {
    try {
        // Starte SPI Bus
        SPI.begin();
        
        // Teensy: Nutze SPI.setClockDivider statt setSpeed
        // Clock Divider 4 = 150MHz / 4 = 37.5 MHz (nah bei 10 MHz)
        // Für stabiler: Divider 8 = 18.75 MHz
        SPI.setClockDivider(SPI_CLOCK_DIV8);
        
        // Setze Pins
        SPI.setMOSI(HAL_SPI_MOSI);
        SPI.setMISO(HAL_SPI_MISO);
        SPI.setSCK(HAL_SPI_SCK);
        
        // Setze CS Pins als Output
        pinMode(HAL_SPI_CS_PRESSURE, OUTPUT);
        pinMode(HAL_SPI_CS_SD, OUTPUT);
        
        // CS Pins default high
        digitalWrite(HAL_SPI_CS_PRESSURE, HIGH);
        digitalWrite(HAL_SPI_CS_SD, HIGH);
        
        Serial.println("[HAL_SPI] SPI Bus initialized");
        return true;
    } catch (...) {
        Serial.println("[HAL_SPI] ERROR: Failed to initialize SPI bus");
        return false;
    }
}

bool spiTransfer(uint8_t csPin, uint8_t* data, uint8_t len) {
    if (!data || len == 0) {
        return false;
    }

    try {
        digitalWrite(csPin, LOW);
        delayMicroseconds(10);
        
        for (uint8_t i = 0; i < len; i++) {
            data[i] = SPI.transfer(data[i]);
        }
        
        delayMicroseconds(10);
        digitalWrite(csPin, HIGH);
        
        return true;
    } catch (...) {
        digitalWrite(csPin, HIGH);
        return false;
    }
}

bool spiWrite(uint8_t csPin, const uint8_t* data, uint8_t len) {
    if (!data || len == 0) {
        return false;
    }

    try {
        digitalWrite(csPin, LOW);
        delayMicroseconds(10);
        
        for (uint8_t i = 0; i < len; i++) {
            SPI.transfer(data[i]);
        }
        
        delayMicroseconds(10);
        digitalWrite(csPin, HIGH);
        
        return true;
    } catch (...) {
        digitalWrite(csPin, HIGH);
        return false;
    }
}

bool spiRead(uint8_t csPin, uint8_t* data, uint8_t len) {
    if (!data || len == 0) {
        return false;
    }

    try {
        digitalWrite(csPin, LOW);
        delayMicroseconds(10);
        
        for (uint8_t i = 0; i < len; i++) {
            data[i] = SPI.transfer(0xFF);  // Lese mit Dummy Bytes
        }
        
        delayMicroseconds(10);
        digitalWrite(csPin, HIGH);
        
        return true;
    } catch (...) {
        digitalWrite(csPin, HIGH);
        return false;
    }
}

bool spiReadRegister(uint8_t csPin, uint8_t reg, uint8_t& value) {
    uint8_t readCmd = reg | 0x80;  // Set MSB für Read-Mode (Konvention)
    
    try {
        digitalWrite(csPin, LOW);
        delayMicroseconds(10);
        
        SPI.transfer(readCmd);
        value = SPI.transfer(0xFF);
        
        delayMicroseconds(10);
        digitalWrite(csPin, HIGH);
        
        return true;
    } catch (...) {
        digitalWrite(csPin, HIGH);
        return false;
    }
}

bool spiWriteRegister(uint8_t csPin, uint8_t reg, uint8_t value) {
    try {
        digitalWrite(csPin, LOW);
        delayMicroseconds(10);
        
        SPI.transfer(reg);
        SPI.transfer(value);
        
        delayMicroseconds(10);
        digitalWrite(csPin, HIGH);
        
        return true;
    } catch (...) {
        digitalWrite(csPin, HIGH);
        return false;
    }
}

} // namespace HAL
