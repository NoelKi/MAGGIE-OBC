#pragma once

#include <cstdint>
#include <SD.h>

/**
 * @file sd_logger.hpp
 * @brief Bordspeicher: jedes Downlink-Frame zusaetzlich auf die SD-Karte
 *
 * Gespeichert werden die 20-Byte-Frames EXAKT so, wie sie ueber die UART gehen
 * (siehe telemetry_hal.hpp), hintereinander ohne Trenner. Dieselbe
 * Parser-Logik wie am Boden liest also auch die Karte.
 *
 *   /B00042/F00000.BIN   Boot 42, erste 10 s
 *   /B00042/F00001.BIN   Boot 42, naechste 10 s ...
 *
 * Alle FILE_PERIOD_MS wird die Datei geschlossen und eine neue begonnen. Ein
 * Stromausfall kostet damit hoechstens die laufende Datei.
 *
 * ---------------------------------------------------------------------------
 * Robustheit - die SD-Karte darf den Loop nie gefaehrden
 * ---------------------------------------------------------------------------
 * 1. RAM-Puffer: push() kopiert nur in einen Ringpuffer (128 KB, > 60 s
 *    Telemetrie). Die Karte wird in service() bedient, hoechstens EINE
 *    SD-Operation (max. 512 Byte) pro Loop.
 * 2. Keine Kartenzugriffe, waehrend ein Motor laeuft: Eine SD-Karte kann
 *    intern fuer 100..250 ms busy sein. In der Zeit liefe ein Motor ueber
 *    sein Encoder-Ziel hinaus. Die Daten warten so lange im Puffer.
 * 3. Fehler = aus: Schlaegt eine Operation fehl, bleibt die Karte bis zum
 *    naechsten Start unangetastet. Telemetrie und alles andere laufen weiter.
 * 4. Haenger-Schutz: Die SD-Bibliothek wartet bei einer defekten Karte bis zu
 *    1 s - das reicht fuer den Watchdog-Reset. Vor jeder Kartenoperation wird
 *    deshalb eine Marke in ein SNVS-Register geschrieben (uebersteht einen
 *    Reset, nicht aber einen Stromausfall). Startet der OBC nach einem
 *    Watchdog-Reset und die Marke steht noch, war die Karte schuld: Sie
 *    bleibt fuer diesen Start aus, statt den OBC in eine Reset-Schleife zu
 *    ziehen.
 */
class SdLogger {
public:
    static constexpr uint32_t FILE_PERIOD_MS = 10000;     ///< neue Datei alle 10 s
    static constexpr uint32_t BUFFER_SIZE    = 128 * 1024; ///< Zweierpotenz (Maske)
    static constexpr uint32_t CHUNK_SIZE     = 512;        ///< ein SD-Sektor

    /**
     * @brief Karte einbinden und das Verzeichnis dieses Starts anlegen.
     * @param boot_count           Nummer dieses Starts (Verzeichnisname)
     * @param after_watchdog_reset true, wenn der letzte Reset vom Watchdog kam
     * @return true, wenn ab jetzt geloggt wird
     */
    bool begin(uint32_t boot_count, bool after_watchdog_reset);

    /// Frame in den RAM-Puffer kopieren. Kein Kartenzugriff, kehrt sofort zurueck.
    void push(const uint8_t* data, uint8_t len);

    /**
     * @brief Karte bedienen - jeden Loop aufrufen.
     * @param io_allowed false, solange ein Motor laeuft (siehe oben, Punkt 2)
     */
    void service(uint32_t now_ms, bool io_allowed);

    bool active() const { return active_; }

private:
    bool     active_     = false;
    uint32_t boot_count_ = 0;
    char     dir_[16]    = {};   ///< Verzeichnis dieses Starts, z.B. "/B00042"
    uint32_t file_index_ = 0;
    uint32_t file_start_ms_ = 0;
    File     file_;
    bool     file_open_  = false;

    // Ringpuffer: head_/tail_ zaehlen fortlaufend, Index = Zaehler & Maske.
    uint32_t head_    = 0;   ///< naechstes freies Byte (push)
    uint32_t tail_    = 0;   ///< naechstes zu schreibende Byte (service)
    uint32_t dropped_ = 0;   ///< verworfene Frames, weil der Puffer voll war
    uint32_t dropped_reported_ = 0;

    bool openNextFile(uint32_t now_ms);
    bool writeChunk();
    bool closeFile();
    void fail(const char* what);
};
