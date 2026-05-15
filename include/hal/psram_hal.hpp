#pragma once

#include <cstdint>
#include <Arduino.h>
#include <SPI.h>

class PSRAMHAL {
public:
    PSRAMHAL(uint8_t cs_pin, uint32_t spi_freq = 20000000UL);

    bool init();
    bool read(uint32_t addr, uint8_t* buf, size_t len);
    bool write(uint32_t addr, const uint8_t* data, size_t len);
    bool readID(uint8_t* mfid, uint8_t* kgd);
    bool isInitialized() const { return initialized_; }

private:
    uint8_t  cs_pin_;
    uint32_t spi_freq_;
    bool     initialized_ = false;

    void csLow();
    void csHigh();
    void sendAddress(uint32_t addr);
};
