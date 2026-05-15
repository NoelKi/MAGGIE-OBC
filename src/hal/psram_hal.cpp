#include "hal/psram_hal.hpp"

// APS6404L SPI command opcodes
static constexpr uint8_t CMD_READ         = 0x03;
static constexpr uint8_t CMD_WRITE        = 0x02;
static constexpr uint8_t CMD_RESET_ENABLE = 0x66;
static constexpr uint8_t CMD_RESET        = 0x99;
static constexpr uint8_t CMD_READ_ID      = 0x9F;

// Expected manufacturer ID
static constexpr uint8_t EXPECTED_MFID = 0x0D;

PSRAMHAL::PSRAMHAL(uint8_t cs_pin, uint32_t spi_freq)
    : cs_pin_(cs_pin), spi_freq_(spi_freq) {
}

void PSRAMHAL::csLow() {
    digitalWrite(cs_pin_, LOW);
}

void PSRAMHAL::csHigh() {
    digitalWrite(cs_pin_, HIGH);
}

void PSRAMHAL::sendAddress(uint32_t addr) {
    SPI.transfer(static_cast<uint8_t>((addr >> 16) & 0xFF));
    SPI.transfer(static_cast<uint8_t>((addr >>  8) & 0xFF));
    SPI.transfer(static_cast<uint8_t>( addr        & 0xFF));
}

bool PSRAMHAL::init() {
    pinMode(cs_pin_, OUTPUT);
    csHigh();

    SPI.begin();

    // Reset sequence
    SPI.beginTransaction(SPISettings(spi_freq_, MSBFIRST, SPI_MODE0));
    csLow();
    SPI.transfer(CMD_RESET_ENABLE);
    csHigh();
    csLow();
    SPI.transfer(CMD_RESET);
    csHigh();
    SPI.endTransaction();

    delayMicroseconds(100);

    // Verify device identity
    uint8_t mfid = 0, kgd = 0;
    if (!readID(&mfid, &kgd)) return false;
    if (mfid != EXPECTED_MFID)  return false;

    initialized_ = true;
    return true;
}

bool PSRAMHAL::readID(uint8_t* mfid, uint8_t* kgd) {
    SPI.beginTransaction(SPISettings(spi_freq_, MSBFIRST, SPI_MODE0));
    csLow();
    SPI.transfer(CMD_READ_ID);
    sendAddress(0x000000);
    *mfid = SPI.transfer(0x00);
    *kgd  = SPI.transfer(0x00);
    csHigh();
    SPI.endTransaction();
    return true;
}

bool PSRAMHAL::read(uint32_t addr, uint8_t* buf, size_t len) {
    if (!initialized_) return false;

    SPI.beginTransaction(SPISettings(spi_freq_, MSBFIRST, SPI_MODE0));
    csLow();
    SPI.transfer(CMD_READ);
    sendAddress(addr);
    for (size_t i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }
    csHigh();
    SPI.endTransaction();
    return true;
}

bool PSRAMHAL::write(uint32_t addr, const uint8_t* data, size_t len) {
    if (!initialized_) return false;

    SPI.beginTransaction(SPISettings(spi_freq_, MSBFIRST, SPI_MODE0));
    csLow();
    SPI.transfer(CMD_WRITE);
    sendAddress(addr);
    for (size_t i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }
    csHigh();
    SPI.endTransaction();
    return true;
}
