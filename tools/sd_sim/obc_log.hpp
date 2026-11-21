// Host-Stub: Log schreibt auf die Konsole (nur mit SD_SIM_VERBOSE=1).
#pragma once
#include <Arduino.h>
class FakeLog {
public:
    void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        if (!verbose) return;
        va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
    }
    void println(const char* s = "") { if (verbose) ::printf("%s\n", s); }
    bool verbose = false;
};
extern FakeLog Log;
