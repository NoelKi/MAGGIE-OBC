// Minimaler Arduino-Ersatz fuer den Host-Test des Downlinks (tools/downlink_sim).
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>

#define SERIAL_8N1 0
#define SERIAL_8N1_RXINV 1

uint32_t millis();

class HardwareSerial {
public:
    virtual ~HardwareSerial() = default;
    virtual void   begin(uint32_t baud, uint16_t format = 0) = 0;
    virtual int    available() = 0;
    virtual int    read() = 0;
    virtual int    availableForWrite() = 0;
    virtual size_t write(const uint8_t* buf, size_t len) = 0;
};
