#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>

/**
 * @file obc_log.hpp
 * @brief Debug-Ausgabe an USB UND an die SD-Karte (LOG.txt)
 *
 * Alle Meldungen der Firmware gehen ueber `Log` statt `Serial`:
 *     Log.printf("INFO  [System]: ...\n");
 * Log schreibt unveraendert an den USB-Monitor und sammelt jede Zeile fuer die
 * SD-Karte (siehe SdLogger::pushLogLine). Zeilen werden bei '\n' abgegeben,
 * laengere als LINE_MAX in Stuecken.
 *
 * Nie blockierend gegenueber der SD-Karte: Die Senke kopiert nur in den
 * RAM-Puffer des SdLoggers.
 */
class ObcLog : public Print {
public:
    static constexpr size_t LINE_MAX = 200;
    using Sink = void (*)(void* ctx, const char* line, size_t len);

    void setSink(Sink sink, void* ctx) { sink_ = sink; ctx_ = ctx; }

    size_t write(uint8_t b) override;
    size_t write(const uint8_t* buffer, size_t size) override;
    using Print::write;

private:
    Sink   sink_ = nullptr;
    void*  ctx_  = nullptr;
    char   line_[LINE_MAX];
    size_t len_  = 0;

    void collect(uint8_t b);
    void emit();
};

extern ObcLog Log;
