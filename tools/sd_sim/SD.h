// SD-Karte als Host-Verzeichnis (tools/sd_sim). Jede Operation zaehlt mit und
// kann kuenstlich verlangsamt oder zum Fehlschlagen gebracht werden.
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>

#define BUILTIN_SDCARD 254
#define FILE_WRITE_BEGIN 2

struct FakeCard {
    std::string root;            ///< Host-Ordner, der die Karte darstellt
    bool     present   = true;
    uint32_t op_delay_us = 0;    ///< jede Operation "dauert" so lange
    bool     fail_write = false;
    uint32_t ops = 0;            ///< Anzahl Kartenoperationen
    uint32_t writes = 0;
};
extern FakeCard g_card;
void fakeAdvanceUs(uint32_t us);   // in sd_sim.cpp

class File {
public:
    File() = default;
    explicit File(FILE* f) : f_(f) {}
    size_t write(const uint8_t* b, size_t n) {
        g_card.ops++; g_card.writes++;
        fakeAdvanceUs(g_card.op_delay_us);
        if (!f_ || g_card.fail_write) { err_ = 1; return 0; }
        return fwrite(b, 1, n, f_);
    }
    void flush() {
        g_card.ops++;
        fakeAdvanceUs(g_card.op_delay_us);
        if (f_) fflush(f_);
    }
    void close() { if (f_) fclose(f_); f_ = nullptr; }
    int getWriteError() { return err_; }
    explicit operator bool() const { return f_ != nullptr; }
private:
    FILE* f_ = nullptr;
    int   err_ = 0;
};

class SDClass {
public:
    bool begin(uint8_t) { g_card.ops++; return g_card.present; }
    bool exists(const char* path);
    bool mkdir(const char* path);
    File open(const char* path, int mode);
};
extern SDClass SD;
