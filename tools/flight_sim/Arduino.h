// Minimaler Arduino-Ersatz fuer den Host-Test der Flug-Trajektorie
// (tools/flight_sim). Nur was arm_hal und arm_flight_player brauchen.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>

#define PROGMEM

uint32_t millis();

class HardwareSerial {
public:
    virtual ~HardwareSerial() = default;
    virtual void   begin(uint32_t baud) = 0;
    virtual int    available() = 0;
    virtual int    read() = 0;
    virtual int    availableForWrite() = 0;
    virtual size_t write(const uint8_t* buf, size_t len) = 0;
};

/// USB-Konsole: sammelt WARN-Zeilen, druckt nur mit FLIGHT_SIM_VERBOSE=1.
class FakeConsole {
public:
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    void println(const char* s = "");
    int  warnings = 0;
    bool verbose  = false;
};
extern FakeConsole Serial;
