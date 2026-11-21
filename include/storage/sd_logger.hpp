#pragma once

#include <cstdint>
#include <cstddef>
#include <SD.h>

/**
 * @file sd_logger.hpp
 * @brief Bordspeicher: ALLES auf die SD-Karte, als lesbare CSV-Dateien
 *
 * ---------------------------------------------------------------------------
 * Ordner und Dateien
 * ---------------------------------------------------------------------------
 *   /MAGGIE/LIESMICH.TXT                    Aufbau und Spalten (einmal angelegt)
 *   /MAGGIE/2026-11-19/                     ein Ordner pro Tag (Teensy-Uhr)
 *   /MAGGIE/2026-11-19/093512_B00042/       ein Start: Uhrzeit + Boot-Nummer
 *       INFO.txt  LOG.txt  EVENTS.csv  SYS.csv  IMU.csv  FORCE_T1.csv
 *       FORCE_T2.csv  MOTOR.csv  ARM_JOINTS.csv  ARM_MODULE_IMU.csv
 *       ARM_POWER.csv  ARM_RCU_IMU.csv  ARM_RAW.csv  CAMERA.csv
 *
 * Jede CSV-Datei beginnt mit den Spalten t_s (Sekunden seit Start, ms genau)
 * und zustand. Spalten und Einheiten: LIESMICH.TXT (Text in sd_logger.cpp).
 *
 * Die Uhr des Teensy steht ohne Knopfzelle an VBAT nach einem Stromausfall
 * auf 2019-01-01 (nur direkt nach dem Flashen stimmt sie). Dann stimmt der Tagesordner nicht - es geht
 * aber nichts verloren, und die Boot-Nummer haelt die Starts auseinander.
 *
 * ---------------------------------------------------------------------------
 * Ablauf
 * ---------------------------------------------------------------------------
 *   push*()   Messwert als kompakten Binaer-Datensatz in den RAM-Ring
 *             (256 KB). Kein Kartenzugriff, kehrt sofort zurueck. Geht schon
 *             vor begin() - die Boot-Meldungen landen so auch auf der Karte.
 *   service() jeden Loop:
 *             1. Datensaetze aus dem Ring in Text umwandeln (reine Rechenzeit,
 *                je Datei ein 2-KB-Textpuffer).
 *             2. HOECHSTENS EINE Kartenoperation: Ordner/Datei anlegen, 512 Byte
 *                schreiben oder eine Datei sichern (sync).
 *             Jede Datei bleibt offen und wird alle SYNC_MS gesichert - ein
 *             Stromausfall kostet hoechstens so viel plus den RAM-Inhalt.
 *
 * ---------------------------------------------------------------------------
 * Robustheit - die SD-Karte darf den Loop nie gefaehrden
 * ---------------------------------------------------------------------------
 * 1. Keine Kartenzugriffe, waehrend ein Motor laeuft (io_allowed = false):
 *    Eine Karte kann intern 100..250 ms busy sein. Der Ring fasst ~10..15 s.
 * 2. Fehler = aus: Schlaegt eine Operation fehl, bleibt die Karte bis zum
 *    naechsten Start unangetastet. Alles andere laeuft weiter.
 * 3. Langsame Karte = aus: Jede Operation wird gemessen. Dauert eine laenger
 *    als HARD_SLOW_US oder SLOW_LIMIT Stueck laenger als SLOW_US (innerhalb
 *    SLOW_WINDOW_MS), wird die Karte abgeschaltet.
 * 4. Haenger: Vor jeder Kartenoperation steht die Brotkrume LOOP_SD
 *    (boot_guard.hpp). Haengt die Karte bis zum Watchdog-Reset, bleibt sie
 *    beim naechsten Start aus.
 * 5. Keine langen Operationen: kein Verzeichnis-Scan, keine Abfrage des
 *    freien Platzes (dauert bei grossen Karten > 1 s), kein Datei-Wechsel.
 */

/// Typ eines Datensatzes im Ring.
enum class SdRec : uint8_t {
    LOG = 1, INFO, EVT_STATE, EVT_TC, EVT_TEXT, IMU, FORCE, MOTOR, SYS, CAM, ARM,
};

/// Zustand fuer SYS/SD - Werte gehen an den Boden, FEST.
enum class SdLogStatus : uint8_t {
    NO_CARD  = 0,   ///< keine Karte gefunden
    SETUP    = 1,   ///< Ordner/Dateien werden angelegt
    ACTIVE   = 2,   ///< schreibt
    FAILED   = 3,   ///< Fehler, aus bis zum naechsten Start (Grund: SdLogFail)
    DISABLED = 4,   ///< nach Watchdog-Reset in der Karte / Minimalmodus aus
    BUFFERING = 5,  ///< vor begin() - sammelt nur im RAM
};

enum class SdLogFail : uint8_t {
    NONE = 0, MKDIR = 1, OPEN = 2, WRITE = 3, SYNC = 4, SLOW = 5, README = 6,
};

// ---------------------------------------------------------------------------
// Nutzdaten der Datensaetze (Rechner-Byteordnung, nur RAM)
// ---------------------------------------------------------------------------
struct SdImu {
    int16_t acc[3];
    int16_t gyro[3];
    int16_t temp;     ///< BMI088 11 Bit, siehe IMUReading::raw_temp
    uint8_t valid;
};

struct SdForce {
    uint8_t sensor;   ///< 1 oder 2
    uint8_t count;    ///< belegte Kanaele
    uint8_t flags;    ///< Bit0 tariert, Bit1 haengt, Bit2 neuer Wert
    uint8_t pad;
    int32_t counts[4];   ///< tariert
    int32_t raw[4];      ///< roh (24 Bit)
};
static constexpr uint8_t SD_FORCE_TARED = 0x01;
static constexpr uint8_t SD_FORCE_STALE = 0x02;
static constexpr uint8_t SD_FORCE_FRESH = 0x04;

struct SdMotor {
    uint8_t motor;    ///< 1 oder 2
    uint8_t state;    ///< DL_MOTOR_STATE_*
    int16_t pwm;
    int32_t position;
};

struct SdCam {
    uint8_t  cam;       ///< 1..4
    uint8_t  flags;     ///< DL_CAM_*
    uint8_t  age_s;     ///< 255 = nie / >= 255 s
    uint8_t  protocol;
    uint16_t features;
    uint16_t probes_failed;
};

/// 1-Hz-Ueberblick. Die SD-eigenen Felder fuellt pushSys() selbst.
struct SdSys {
    uint32_t rtc;            ///< Unix-Zeit der Teensy-Uhr
    uint8_t  subsys;         ///< DL_SUBSYS_*
    uint8_t  healthy;
    uint8_t  rexus_raw;      ///< DL_REXUS_* (Rohpegel)
    uint8_t  uplink_last_op;
    uint32_t uplink_rx;
    uint16_t uplink_ok;
    uint16_t uplink_bad;
    uint32_t arm_ok;           ///< gueltige Frames seit Start
    uint32_t arm_crc_errors;   ///< seit Start
    uint32_t arm_unknown_ids;  ///< seit Start
    uint8_t  arm_alive;
    uint8_t  arm_freefall;
    uint8_t  arm_last_msg;
    uint8_t  arm_last_node;
    uint8_t  traj_phase;
    uint8_t  flight_phase;
    uint8_t  flight_flags;
    uint8_t  flight_switch;
    uint8_t  cams_alive;
    uint8_t  sd_fill_pct;
    uint16_t arm_tx_dropped;
    uint32_t dl_dropped_status;   ///< Downlink-Verluste je Stufe (DownlinkPrio)
    uint32_t dl_dropped_motion;
    uint32_t dl_dropped_bulk;
    uint32_t sd_dropped;
    uint32_t sd_kb;
    uint32_t loop_max_us;
    uint32_t loops;
};

class SdLogger {
public:
    static constexpr uint32_t BUFFER_SIZE  = 256 * 1024;  ///< Zweierpotenz (Maske)
    static constexpr uint32_t CHUNK_SIZE   = 512;         ///< ein SD-Sektor
    static constexpr uint32_t TEXT_BUF     = 2048;        ///< Textpuffer je Datei
    static constexpr uint32_t SYNC_MS      = 2000;        ///< jede Datei so oft sichern
    static constexpr uint32_t SLOW_US      = 150000;      ///< "langsame" Operation
    static constexpr uint32_t HARD_SLOW_US = 600000;      ///< sofort aus
    static constexpr uint8_t  SLOW_LIMIT   = 3;
    static constexpr uint32_t SLOW_WINDOW_MS = 30000;
    static constexpr uint8_t  FORMAT_PER_CALL = 48;       ///< Datensaetze je service()
    static constexpr uint8_t  SD_BEGIN_TRIES  = 3;        ///< SD.begin()-Versuche beim Start
    static constexpr uint32_t SD_BEGIN_RETRY_MS = 250;

    enum FileId : uint8_t {
        F_INFO, F_LOG, F_EVENTS, F_SYS, F_IMU, F_FORCE_T1, F_FORCE_T2, F_MOTOR,
        F_ARM_JOINTS, F_ARM_MODULE_IMU, F_ARM_POWER, F_ARM_RCU_IMU, F_ARM_RAW, F_CAMERA,
        F_COUNT
    };

    /**
     * @brief Karte einbinden. Ordner und Dateien legt service() an.
     * @param boot_count Nummer dieses Starts (Ordnername)
     * @param disabled   true = Karte bleibt aus (Boot-Guard: Haenger/Minimalmodus)
     * @return true, wenn eine Karte gefunden wurde
     */
    bool begin(uint32_t boot_count, bool disabled);

    /// Aktuellen Missionszustand fuer die Spalte "zustand" merken.
    void setState(uint8_t state) { state_ = state; }

    // --- Datensaetze (nie blockierend) -------------------------------------
    void pushLogLine(const char* text, size_t len);
    /// Eine Zeile fuer INFO.txt (printf-Format, ohne '\n').
    void infof(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    void pushStateEvent(uint8_t state, uint8_t prev, uint8_t signals, uint8_t prev_signals);
    void pushTelecommand(uint8_t opcode, int16_t arg, bool accepted);
    /// Freies Ereignis fuer EVENTS.csv: name (ohne Komma) + Details (printf).
    void eventf(const char* name, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
    void pushImu(const SdImu& r)     { push(SdRec::IMU, &r, sizeof(r)); }
    void pushForce(const SdForce& r) { push(SdRec::FORCE, &r, sizeof(r)); }
    void pushMotor(const SdMotor& r) { push(SdRec::MOTOR, &r, sizeof(r)); }
    void pushCam(const SdCam& r)     { push(SdRec::CAM, &r, sizeof(r)); }
    void pushSys(SdSys& r);
    /// Ein Arm-Frame. kind = ArmTapKind (RX, TX, RX_BAD_CRC, RX_BAD_ID);
    /// gueltige Frames ohne CRC, verworfene mit allen Rohbytes.
    void pushArm(uint8_t kind, const uint8_t* frame, uint8_t len);

    /**
     * @brief Karte bedienen - jeden Loop aufrufen.
     * @param io_allowed false, solange ein Motor laeuft
     */
    void service(uint32_t now_ms, bool io_allowed);

    // --- Status fuer Downlink und SYS.csv -----------------------------------
    SdLogStatus status() const { return status_; }
    SdLogFail   failReason() const { return fail_; }
    uint32_t writtenKb() const { return static_cast<uint32_t>(written_ >> 10); }
    uint32_t dropped() const { return dropped_; }
    uint8_t  fillPercent() const;
    const char* sessionPath() const { return session_; }

private:
    enum class Setup : uint8_t { ROOT, README, DATE, SESSION, OPEN, RUN };

    SdLogStatus status_ = SdLogStatus::BUFFERING;
    SdLogFail   fail_   = SdLogFail::NONE;
    Setup    setup_  = Setup::ROOT;
    uint8_t  open_next_ = 0;
    uint8_t  state_  = 0;
    uint32_t boot_count_ = 0;

    char date_dir_[24] = {};   ///< "/MAGGIE/2026-11-19"
    char session_[48]  = {};   ///< "/MAGGIE/2026-11-19/093512_B00042"

    File     files_[F_COUNT];
    uint16_t tlen_[F_COUNT] = {};        ///< Bytes im Textpuffer
    bool     dirty_[F_COUNT] = {};       ///< geschrieben, aber noch nicht gesichert
    uint32_t last_sync_[F_COUNT] = {};
    uint8_t  sync_rr_ = 0;

    // Ring: head_/tail_ zaehlen fortlaufend, Index = Zaehler & Maske.
    uint32_t head_ = 0;
    uint32_t tail_ = 0;
    uint32_t dropped_ = 0;
    uint64_t written_ = 0;

    uint8_t  slow_count_ = 0;
    uint32_t slow_first_ms_ = 0;

    bool acceptsData() const {
        return status_ == SdLogStatus::BUFFERING || status_ == SdLogStatus::SETUP ||
               status_ == SdLogStatus::ACTIVE;
    }
    void push(SdRec type, const void* payload, uint8_t len);
    void pushText(SdRec type, const char* text, size_t len);

    void formatPending();
    bool formatRecord(const uint8_t* rec, uint8_t len);
    bool append(uint8_t file, const char* text, size_t len);
    bool room(uint8_t file, size_t len) const { return tlen_[file] + len <= TEXT_BUF; }
    void addHeaders();

    void setupStep(uint32_t now_ms);
    void runStep(uint32_t now_ms);
    bool writeFile(uint8_t file, uint32_t n, uint32_t now_ms);
    /// Dauer einer Kartenoperation bewerten. false = Karte wurde abgeschaltet.
    bool checkDuration(uint32_t start_us, uint32_t now_ms, const char* what);
    void fail(SdLogFail why, const char* what);
};
