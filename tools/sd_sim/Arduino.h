// Minimaler Arduino-Ersatz fuer den Host-Test des SD-Loggers (tools/sd_sim).
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <cstring>

#define DMAMEM
#define HIGH 1
#define LOW 0
#define OUTPUT 1

uint32_t millis();
uint32_t micros();
unsigned long rtc_get();
inline void delay(uint32_t) {}

class HardwareSerial {
public:
    virtual ~HardwareSerial() = default;
    virtual void   begin(uint32_t baud) = 0;
    virtual int    available() = 0;
    virtual int    read() = 0;
    virtual int    availableForWrite() = 0;
    virtual size_t write(const uint8_t* buf, size_t len) = 0;
};
