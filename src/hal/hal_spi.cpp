#include "hal_spi.hpp"
#include "hal_config.hpp"

namespace HAL {

bool spiInit() {
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);

    SPI.setMOSI(HAL_SPI_MOSI);
    SPI.setMISO(HAL_SPI_MISO);
    SPI.setSCK(HAL_SPI_SCK);

    pinMode(HAL_SPI_CS_PRESSURE, OUTPUT);
    pinMode(HAL_SPI_CS_SD, OUTPUT);

    digitalWrite(HAL_SPI_CS_PRESSURE, HIGH);
    digitalWrite(HAL_SPI_CS_SD, HIGH);

    Serial.println("[HAL_SPI] SPI Bus initialized");
    return true;
}

bool spiTransfer(uint8_t csPin, uint8_t* data, uint8_t len) {
    if (!data || len == 0) return false;

    digitalWrite(csPin, LOW);
    delayMicroseconds(10);

    for (uint8_t i = 0; i < len; i++) {
        data[i] = SPI.transfer(data[i]);
    }

    delayMicroseconds(10);
    digitalWrite(csPin, HIGH);
    return true;
}

bool spiWrite(uint8_t csPin, const uint8_t* data, uint8_t len) {
    if (!data || len == 0) return false;

    digitalWrite(csPin, LOW);
    delayMicroseconds(10);

    for (uint8_t i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }

    delayMicroseconds(10);
    digitalWrite(csPin, HIGH);
    return true;
}

bool spiRead(uint8_t csPin, uint8_t* data, uint8_t len) {
    if (!data || len == 0) return false;

    digitalWrite(csPin, LOW);
    delayMicroseconds(10);

    for (uint8_t i = 0; i < len; i++) {
        data[i] = SPI.transfer(0xFF);
    }

    delayMicroseconds(10);
    digitalWrite(csPin, HIGH);
    return true;
}

bool spiReadRegister(uint8_t csPin, uint8_t reg, uint8_t& value) {
    uint8_t readCmd = reg | 0x80;

    digitalWrite(csPin, LOW);
    delayMicroseconds(10);

    SPI.transfer(readCmd);
    value = SPI.transfer(0xFF);

    delayMicroseconds(10);
    digitalWrite(csPin, HIGH);
    return true;
}

bool spiWriteRegister(uint8_t csPin, uint8_t reg, uint8_t value) {
    digitalWrite(csPin, LOW);
    delayMicroseconds(10);

    SPI.transfer(reg);
    SPI.transfer(value);

    delayMicroseconds(10);
    digitalWrite(csPin, HIGH);
    return true;
}

} // namespace HAL
