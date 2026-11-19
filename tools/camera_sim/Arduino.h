// Minimaler Arduino-Ersatz fuer den Host-Test des Kameratreibers
// (tools/camera_sim). Nur was camera_hal braucht.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>

#define HIGH 1
#define LOW 0
#define OUTPUT 1

uint32_t millis();
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);

class HardwareSerial {
public:
    virtual ~HardwareSerial() = default;
    virtual void   begin(uint32_t baud) = 0;
    virtual int    available() = 0;
    virtual int    read() = 0;
    virtual int    availableForWrite() = 0;
    virtual size_t write(const uint8_t* buf, size_t len) = 0;
};

class FakeConsole {
public:
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        if (!verbose) return;
        va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
    }
    void println(const char* s = "") { if (verbose) ::printf("%s\n", s); }
    bool verbose = false;
};
extern FakeConsole Serial;
