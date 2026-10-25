#include "storage/sd_logger.hpp"
#include <cstring>

static_assert((SdLogger::BUFFER_SIZE & (SdLogger::BUFFER_SIZE - 1)) == 0,
              "BUFFER_SIZE muss eine Zweierpotenz sein");
static constexpr uint32_t MASK = SdLogger::BUFFER_SIZE - 1;

// RAM2 (OCRAM, 512 KB) - der Puffer ist zu gross fuer RAM1 und braucht dort
// keinen schnellen Zugriff.
DMAMEM static uint8_t ring_[SdLogger::BUFFER_SIZE];

// ---------------------------------------------------------------------------
// Haenger-Schutz (siehe sd_logger.hpp, Punkt 4)
// ---------------------------------------------------------------------------
// SNVS_LPGPR0 liegt in der stromsparenden SNVS-Domaene und behaelt seinen Wert
// ueber einen Watchdog-Reset hinweg. Die Firmware nutzt das Register sonst nicht.
static constexpr uint32_t SD_BUSY_MARK = 0x53444255;   // "SDBU"
static inline void markBusy() { SNVS_LPGPR0 = SD_BUSY_MARK; }
static inline void markIdle() { SNVS_LPGPR0 = 0; }

bool SdLogger::begin(uint32_t boot_count, bool after_watchdog_reset) {
    boot_count_ = boot_count;

    const bool sd_hung_last_time = after_watchdog_reset && SNVS_LPGPR0 == SD_BUSY_MARK;
    markIdle();
    if (sd_hung_last_time) {
        Serial.println("WARN  [SdLogger]: Letzter Watchdog-Reset kam aus einem "
                       "SD-Zugriff - Karte bleibt fuer diesen Start aus.");
        return false;
    }

    // Haelt das Register die Marke ueberhaupt? Wenn nicht, wirkt der
    // Haenger-Schutz nicht - das soll beim Bodentest auffallen.
    markBusy();
    if (SNVS_LPGPR0 != SD_BUSY_MARK) {
        Serial.println("WARN  [SdLogger]: SNVS_LPGPR0 nicht beschreibbar - "
                       "Haenger-Schutz der SD-Karte wirkt NICHT.");
    }
    const bool ok = SD.begin(BUILTIN_SDCARD);
    markIdle();
    if (!ok) {
        Serial.println("WARN  [SdLogger]: Keine SD-Karte gefunden - Bordspeicher aus.");
        return false;
    }

    // Verzeichnis dieses Starts. Gibt es den Namen schon (Boot-Zaehler wurde
    // zurueckgesetzt, Karte aus einem anderen Aufbau), wird ein Suffix
    // angehaengt - vorhandene Daten werden nie ueberschrieben.
    bool dir_ok = false;
    for (uint8_t suffix = 0; suffix < 10 && !dir_ok; suffix++) {
        if (suffix == 0) {
            snprintf(dir_, sizeof(dir_), "/B%05lu", static_cast<unsigned long>(boot_count_));
        } else {
            snprintf(dir_, sizeof(dir_), "/B%05lu_%u", static_cast<unsigned long>(boot_count_), suffix);
        }
        markBusy();
        dir_ok = !SD.exists(dir_) && SD.mkdir(dir_);
        markIdle();
    }
    if (!dir_ok) {
        Serial.printf("WARN  [SdLogger]: Kein Verzeichnis fuer Boot %lu anlegbar - Bordspeicher aus.\n",
                      static_cast<unsigned long>(boot_count_));
        return false;
    }

    active_ = true;
    Serial.printf("INFO  [SdLogger]: Bordspeicher aktiv, Verzeichnis %s, "
                  "neue Datei alle %lu s.\n",
                  dir_, static_cast<unsigned long>(FILE_PERIOD_MS / 1000));
    return true;
}

void SdLogger::push(const uint8_t* data, uint8_t len) {
    if (!active_) return;

    if (BUFFER_SIZE - (head_ - tail_) < len) {
        dropped_++;   // Puffer voll: lieber dieses Frame verlieren als blockieren
        return;
    }
    for (uint8_t i = 0; i < len; i++) ring_[(head_ + i) & MASK] = data[i];
    head_ += len;
}

void SdLogger::service(uint32_t now_ms, bool io_allowed) {
    if (!active_ || !io_allowed) return;

    // Hoechstens EINE Kartenoperation pro Aufruf - jede kann dauern.
    if (!file_open_) {
        if (!openNextFile(now_ms)) fail("Datei anlegen");
        return;
    }

    const uint32_t buffered = head_ - tail_;
    const bool     rotate   = now_ms - file_start_ms_ >= FILE_PERIOD_MS;

    if (buffered >= CHUNK_SIZE || (rotate && buffered > 0)) {
        if (!writeChunk()) fail("Schreiben");
        return;
    }

    if (rotate) {
        if (!closeFile()) fail("Schliessen");
    }
}

bool SdLogger::openNextFile(uint32_t now_ms) {
    char path[32];
    snprintf(path, sizeof(path), "%s/F%05lu.BIN", dir_,
             static_cast<unsigned long>(file_index_));

    markBusy();
    file_ = SD.open(path, FILE_WRITE_BEGIN);
    markIdle();
    if (!file_) return false;

    file_open_     = true;
    file_start_ms_ = now_ms;
    return true;
}

bool SdLogger::writeChunk() {
    // Nur bis zum Pufferende schreiben - der Rest kommt im naechsten Loop.
    uint32_t n = head_ - tail_;
    if (n > CHUNK_SIZE) n = CHUNK_SIZE;
    const uint32_t start = tail_ & MASK;
    if (n > BUFFER_SIZE - start) n = BUFFER_SIZE - start;

    markBusy();
    const size_t written = file_.write(&ring_[start], n);
    markIdle();
    if (written != n) return false;

    tail_ += n;
    return true;
}

bool SdLogger::closeFile() {
    // close() schreibt Groesse und Verzeichniseintrag - erst danach ist die
    // Datei bei einem Stromausfall sicher.
    markBusy();
    file_.close();
    markIdle();

    file_open_ = false;
    file_index_++;

    if (dropped_ != dropped_reported_) {
        Serial.printf("WARN  [SdLogger]: %lu Frames verworfen (Puffer voll).\n",
                      static_cast<unsigned long>(dropped_ - dropped_reported_));
        dropped_reported_ = dropped_;
    }
    return true;
}

void SdLogger::fail(const char* what) {
    // Die Karte NICHT mehr anfassen - auch kein close(): Bei einer defekten
    // Karte wuerde genau das wieder bis zu 1 s haengen.
    active_ = false;
    Serial.printf("WARN  [SdLogger]: SD-Fehler beim %s (Datei %lu) - "
                  "Bordspeicher fuer diesen Start aus.\n",
                  what, static_cast<unsigned long>(file_index_));
}
