#include "storage/sd_logger.hpp"
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include "boot_guard.hpp"
#include "obc_log.hpp"
#include "hal/arm_hal.hpp"
#include "hal/telemetry_hal.hpp"
#include "hal/uplink_hal.hpp"
#include "hal/rexus_hal.hpp"
#include "statemachine/state_machine.hpp"

static_assert((SdLogger::BUFFER_SIZE & (SdLogger::BUFFER_SIZE - 1)) == 0,
              "BUFFER_SIZE muss eine Zweierpotenz sein");
static constexpr uint32_t MASK = SdLogger::BUFFER_SIZE - 1;
static constexpr uint8_t  REC_HDR = 7;   // len, type, state, t_ms (4, LE)

// RAM2 (OCRAM) - zu gross fuer RAM1, braucht dort keinen schnellen Zugriff.
DMAMEM static uint8_t ring_[SdLogger::BUFFER_SIZE];
DMAMEM static char    tbuf_[SdLogger::F_COUNT][SdLogger::TEXT_BUF];

static const char* const FILE_NAMES[SdLogger::F_COUNT] = {
    "INFO.txt", "LOG.txt", "EVENTS.csv", "SYS.csv", "IMU.csv", "FORCE_T1.csv",
    "FORCE_T2.csv", "MOTOR.csv", "ARM_JOINTS.csv", "ARM_MODULE_IMU.csv",
    "ARM_POWER.csv", "ARM_RCU_IMU.csv", "ARM_RAW.csv", "CAMERA.csv",
};

static const char* const FILE_HEADERS[SdLogger::F_COUNT] = {
    nullptr,
    nullptr,
    "t_s,zustand,ereignis,details\n",
    "t_s,zustand,uhrzeit,subsysteme_hex,gesund,rexus_roh_hex,uplink_bytes,uplink_ok,"
    "uplink_fehler,letzter_opcode_hex,arm_link,arm_frames_ok,arm_frames_fehler,arm_letzte_id_hex,"
    "arm_letztes_modul,arm_crc_fehler,arm_id_unbekannt,arm_freefall,arm_tx_verworfen,arm_traj_phase,arm_flug_phase,"
    "arm_flug_flags_hex,flug_schalter,kameras_aktiv,dl_verworfen_status,dl_verworfen_gelenke,"
    "dl_verworfen_rest,sd_puffer_proz,"
    "sd_verworfen,sd_kb,loop_max_ms,loops_pro_s\n",
    "t_s,zustand,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,gueltig\n",
    "t_s,zustand,x,y,z,roh_x,roh_y,roh_z,tariert,haengt,neuer_wert\n",
    "t_s,zustand,a,b,c,d,roh_a,roh_b,roh_c,roh_d,tariert,haengt,neuer_wert\n",
    "t_s,zustand,motor,position_counts,pwm,an,encoder_ok,dreht,drehung_fehlgeschlagen\n",
    "t_s,zustand,modul,gelenkzustand_hex,winkel_grad,winkel_urad\n",
    "t_s,zustand,modul,ctrl1_xl_hex,ctrl2_g_hex,ax_roh,ay_roh,az_roh,gx_roh,gy_roh,gz_roh\n",
    "t_s,zustand,modul,bus_v,stepper_v,mcu_temp_c,imu_temp_c\n",
    "t_s,zustand,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n",
    "t_s,zustand,richtung,modul,id_hex,name,payload_hex\n",
    "t_s,zustand,kamera,antwortet,sek_seit_antwort,protokoll,features_hex,start_moeglich,"
    "stop_moeglich,letzter_befehl,fehlversuche,rx_irgendwas,rx_muell\n",
};

static const char README_TEXT[] =
"MAGGIE Bordspeicher\r\n"
"===================\r\n"
"\r\n"
"Aufbau:  /MAGGIE/<Datum>/<Uhrzeit>_B<Boot-Nr>/\r\n"
"  Ein Ordner pro Tag, darin ein Ordner pro Start des OBC.\r\n"
"  Das Datum kommt von der Teensy-Uhr. Ohne Knopfzelle steht sie nach einem\r\n"
"  Stromausfall auf 2019-01-01 (nur nach dem Flashen stimmt sie) - dann ist der Tag\r\n"
"  falsch. Die Boot-Nummer zaehlt jeden Start und ist eindeutig.\r\n"
"\r\n"
"Alle CSV-Dateien: Komma als Trenner, Punkt als Dezimalzeichen.\r\n"
"  Excel: Daten > Aus Text/CSV.  Python: pandas.read_csv(datei).\r\n"
"  Spalte t_s = Sekunden seit Start des OBC (ms genau) - in ALLEN Dateien\r\n"
"  gleich, damit lassen sich die Dateien zeitlich uebereinanderlegen.\r\n"
"  Spalte zustand = Missionszustand in dem Moment (PRE_LAUNCH, TEST, ...).\r\n"
"\r\n"
"Dateien eines Starts:\r\n"
"  INFO.txt        Boot-Nr., Firmware-Stand, Reset-Grund, Uhrzeit beim Start\r\n"
"  LOG.txt         alle Meldungen wie im seriellen Monitor, mit Zeit\r\n"
"  EVENTS.csv      Zeitleiste: Zustandswechsel, REXUS-Signale, jedes\r\n"
"                  Telecommand (angenommen/abgewiesen), Warnungen, Arm-Ereignisse\r\n"
"  SYS.csv         1x pro Sekunde: Ueberblick ueber alle Subsysteme, Uplink,\r\n"
"                  Arm-Link, Kameras, SD-Puffer, Loop-Dauer und die seit dem\r\n"
"                  Start im Downlink verworfenen Frames je Stufe (Status /\r\n"
"                  Gelenkwinkel / Rest) - Status sollte immer 0 bleiben\r\n"
"  IMU.csv         OBC-IMU (BMI088), 100 Hz. Beschleunigung in g (+-3 g),\r\n"
"                  Drehrate in Grad/s (+-500), Chiptemperatur in Grad C\r\n"
"  FORCE_T1.csv    Kraftsensor 1 (X/Y/Z), jede Wandlung. x/y/z = tarierte\r\n"
"                  HX711-Counts (noch nicht in Newton - Kalibrierfaktor siehe\r\n"
"                  Server), roh_* = Wandlerwert ohne Nullabgleich\r\n"
"  FORCE_T2.csv    Kraftsensor 2 (Zellen A/B/C/D), wie oben\r\n"
"  MOTOR.csv       beide HDRM-Motoren, 100 Hz: Encoder-Position, PWM, Flags\r\n"
"  ARM_JOINTS.csv  jede Gelenkmeldung der RCU: Zustand (roh, hex) und Winkel\r\n"
"  ARM_MODULE_IMU.csv  IMU jedes Gelenkmoduls, Rohwerte (ICD 3.3.2.2;\r\n"
"                  Standard +-4 g = 8192 Counts/g, +-125 Grad/s = 262 Counts/(Grad/s);\r\n"
"                  ctrl1_xl/ctrl2_g = Konfiguration der IMU)\r\n"
"  ARM_POWER.csv   Spannungen (V) und Temperaturen (Grad C) jedes Moduls. Die RCU\r\n"
"                  schickt beide getrennt - pro Zeile ist eine Haelfte leer.\r\n"
"  ARM_RCU_IMU.csv IMU der RCU in g und Grad/s\r\n"
"  ARM_RAW.csv     JEDES gueltige Frame zur/von der RCU, roh (RX = empfangen,\r\n"
"                  TX = gesendet) - auch Meldungen, die der OBC nicht auswertet.\r\n"
"                  Dazu bis zu 20/s verworfene Empfangs-Kandidaten zur Diagnose:\r\n"
"                  RX_CRC_FEHLER (payload_hex = Payload + CRC, so wie empfangen)\r\n"
"                  und RX_ID_UNBEKANNT (payload_hex = was nach dem Kopf im Puffer lag)\r\n"
"  CAMERA.csv      1x pro Sekunde je Kamera: antwortet, Features, letzter Befehl\r\n"
"\r\n"
"Die Dateien bleiben offen und werden alle 2 s gesichert. Bei Stromausfall\r\n"
"fehlen hoechstens die letzten Sekunden. Waehrend ein Motor laeuft, schreibt\r\n"
"der OBC nicht auf die Karte (die Daten warten im RAM).\r\n";

// ---------------------------------------------------------------------------
// Textzeile ohne printf-Fliesskomma (schnell, keine Heap-Nutzung)
// ---------------------------------------------------------------------------
namespace {

struct Line {
    char     b[320];
    uint16_t n = 0;

    void c(char ch) { if (n < sizeof(b) - 1) b[n++] = ch; }
    void s(const char* str) { while (*str) c(*str++); }
    void s(const char* str, size_t len) { for (size_t i = 0; i < len; i++) c(str[i]); }
    void comma() { c(','); }
    void nl() { c('\n'); }

    void u(uint64_t v) {
        char t[21];
        int k = 0;
        do { t[k++] = static_cast<char>('0' + v % 10); v /= 10; } while (v);
        while (k) c(t[--k]);
    }
    void i(int64_t v) {
        if (v < 0) { c('-'); u(static_cast<uint64_t>(-v)); } else { u(static_cast<uint64_t>(v)); }
    }
    /// v ist mit 10^dec skaliert: fix(12345, 3) -> "12.345"
    void fix(int64_t v, uint8_t dec) {
        if (v < 0) { c('-'); v = -v; }
        uint64_t p = 1;
        for (uint8_t k = 0; k < dec; k++) p *= 10;
        u(static_cast<uint64_t>(v) / p);
        if (!dec) return;
        c('.');
        const uint64_t r = static_cast<uint64_t>(v) % p;
        for (uint64_t q = p / 10; q > 0; q /= 10) c(static_cast<char>('0' + (r / q) % 10));
    }
    void hex(uint32_t v, uint8_t digits) {
        static const char H[] = "0123456789ABCDEF";
        for (int k = digits - 1; k >= 0; k--) c(H[(v >> (4 * k)) & 0xF]);
    }
    void flag(bool v) { c(v ? '1' : '0'); }
    /// CSV-Feld in Anfuehrungszeichen (Kommas im Text erlaubt).
    void quoted(const char* str, size_t len) {
        c('"');
        for (size_t k = 0; k < len; k++) {
            if (str[k] == '"') c('"');
            c(str[k]);
        }
        c('"');
    }
};

/// raw * num / den, kaufmaennisch gerundet.
int64_t scaled(int64_t raw, int64_t num, int64_t den) {
    const int64_t x = raw * num;
    return x >= 0 ? (x + den / 2) / den : -((-x + den / 2) / den);
}

uint16_t rdU16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
int16_t  rdI16(const uint8_t* p) { return static_cast<int16_t>(rdU16(p)); }
uint32_t rdU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

const char* stateName(uint8_t s) {
    return StateMachine::toString(static_cast<MissionState>(s));
}

void rowStart(Line& l, uint32_t t_ms, uint8_t state) {
    l.fix(t_ms, 3);
    l.comma();
    l.s(stateName(state));
    l.comma();
}

const char* opcodeName(uint8_t op) {
    switch (static_cast<UplinkOpcode>(op)) {
        case UplinkOpcode::MOTOR_OFF:        return "MOTOR_OFF";
        case UplinkOpcode::MOTOR_ON:         return "MOTOR_ON";
        case UplinkOpcode::MOTOR_ZERO:       return "MOTOR_ZERO";
        case UplinkOpcode::MOTOR_TURN:       return "MOTOR_TURN";
        case UplinkOpcode::MOTOR_GOTO:       return "MOTOR_GOTO";
        case UplinkOpcode::MOTOR2_OFF:       return "MOTOR2_OFF";
        case UplinkOpcode::MOTOR2_ON:        return "MOTOR2_ON";
        case UplinkOpcode::MOTOR2_ZERO:      return "MOTOR2_ZERO";
        case UplinkOpcode::MOTOR2_TURN:      return "MOTOR2_TURN";
        case UplinkOpcode::MOTOR2_GOTO:      return "MOTOR2_GOTO";
        case UplinkOpcode::TEST_ENTER:       return "TEST_ENTER";
        case UplinkOpcode::TEST_EXIT:        return "TEST_EXIT";
        case UplinkOpcode::STATE_RESET:      return "STATE_RESET";
        case UplinkOpcode::ARM_FLIGHT_ENABLE: return "ARM_FLIGHT_ENABLE";
        case UplinkOpcode::FORCE_TARE:       return "FORCE_TARE";
        case UplinkOpcode::ARM_STOP:         return "ARM_STOP";
        case UplinkOpcode::ARM_INHIBIT:      return "ARM_INHIBIT";
        case UplinkOpcode::ARM_RELEASE:      return "ARM_RELEASE";
        case UplinkOpcode::ARM_JOG:          return "ARM_JOG";
        case UplinkOpcode::ARM_SET_VELOCITY: return "ARM_SET_VELOCITY";
        case UplinkOpcode::ARM_BLINK:        return "ARM_BLINK";
        case UplinkOpcode::ARM_TRAJ_TEST:    return "ARM_TRAJ_TEST";
        case UplinkOpcode::CAMERA:           return "CAMERA";
        default:                             return "UNBEKANNT";
    }
}

const char* armMsgName(uint8_t id) {
    switch (static_cast<ArmMsg>(id)) {
        case ArmMsg::STOP:                 return "STOP";
        case ArmMsg::INHIBIT:              return "INHIBIT";
        case ArmMsg::RELEASE:              return "RELEASE";
        case ArmMsg::PAUSE:                return "PAUSE";
        case ArmMsg::CONTINUE:             return "CONTINUE";
        case ArmMsg::STEPPER_MOVE_REL:     return "MOVE_REL";
        case ArmMsg::STEPPER_MOVE_ABS:     return "MOVE_ABS";
        case ArmMsg::STEPPER_MOVE_STEPS:   return "MOVE_STEPS";
        case ArmMsg::RESET:                return "RESET";
        case ArmMsg::STEPPER_SET_VELOCITY: return "SET_VELOCITY";
        case ArmMsg::SET_HOME:             return "SET_HOME";
        case ArmMsg::INITIATE_LSS_SCAN:    return "LSS_SCAN";
        case ArmMsg::SET_NODE_ID:          return "SET_NODE_ID";
        case ArmMsg::BLINK:                return "BLINK";
        case ArmMsg::JOINT_STATE:          return "JOINT_STATE";
        case ArmMsg::IMU_DATA:             return "IMU_DATA";
        case ArmMsg::VOLTAGE_DATA:         return "VOLTAGE_DATA";
        case ArmMsg::TEMPERATURE_DATA:     return "TEMPERATURE_DATA";
        case ArmMsg::REFERENCE_ANGLE_DATA: return "REFERENCE_ANGLE";
        case ArmMsg::IDENTIFIED:           return "IDENTIFIED";
        case ArmMsg::RCU_IMU_DATA:         return "RCU_IMU_DATA";
        case ArmMsg::RCU_FREEFALL_EVENT:   return "RCU_FREEFALL";
    }
    return "UNBEKANNT";
}

void signalText(Line& l, uint8_t sig) {
    l.s("LO=");   l.flag(sig & DL_REXUS_L0);
    l.s(" SOE="); l.flag(sig & DL_REXUS_SOE);
    l.s(" SODS="); l.flag(sig & DL_REXUS_SODS);
}

/// Unix-Zeit -> Datum (Howard Hinnant, days_from_civil invertiert).
void civil(uint32_t unix_s, int& y, unsigned& m, unsigned& d) {
    const int32_t z = static_cast<int32_t>(unix_s / 86400) + 719468;
    const int32_t era = z / 146097;
    const uint32_t doe = static_cast<uint32_t>(z - era * 146097);
    const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const uint32_t mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp < 10 ? mp + 3 : mp - 9;
    y = static_cast<int>(yoe) + era * 400 + (m <= 2 ? 1 : 0);
}

}  // namespace

// ---------------------------------------------------------------------------
// Start
// ---------------------------------------------------------------------------

bool SdLogger::begin(uint32_t boot_count, bool disabled) {
    boot_count_ = boot_count;

    if (disabled) {
        status_ = SdLogStatus::DISABLED;
        head_ = tail_ = 0;
        Log.println("WARN  [SdLogger]: SD-Karte fuer diesen Start AUS (Boot-Guard: "
                    "Haenger beim letzten Mal oder Minimalmodus).");
        return false;
    }

    // Mehrere Versuche: Nach einem Neustart OHNE Stromunterbrechung (Flashen,
    // Watchdog) steckt die Karte manchmal noch mitten in der letzten
    // Schreiboperation und antwortet auf die erste Initialisierung nicht
    // (Bodentest 19.11.2026: Boot 123 und 133 "keine Karte").
    bool ok = false;
    for (uint8_t attempt = 1; attempt <= SD_BEGIN_TRIES && !ok; attempt++) {
        {
            bootguard::Scope guard(Stage::INIT_SD);
            ok = SD.begin(BUILTIN_SDCARD);
        }
        bootguard::watchdogFeed();
        if (!ok && attempt < SD_BEGIN_TRIES) {
            Log.printf("INFO  [SdLogger]: SD-Karte antwortet nicht (Versuch %u/%u) - erneut.\n",
                       attempt, SD_BEGIN_TRIES);
            delay(SD_BEGIN_RETRY_MS);
            bootguard::watchdogFeed();
        }
    }
    if (!ok) {
        status_ = SdLogStatus::NO_CARD;
        head_ = tail_ = 0;
        Log.println("WARN  [SdLogger]: Keine SD-Karte gefunden - Bordspeicher aus.");
        return false;
    }

    // Ordnernamen aus der Teensy-Uhr.
    const uint32_t now_s = rtc_get();
    int y; unsigned mo, d;
    civil(now_s, y, mo, d);
    const uint32_t sec = now_s % 86400;
    snprintf(date_dir_, sizeof(date_dir_), "/MAGGIE/%04d-%02u-%02u", y, mo, d);
    snprintf(session_, sizeof(session_), "%s/%02lu%02lu%02lu_B%05lu", date_dir_,
             static_cast<unsigned long>(sec / 3600), static_cast<unsigned long>((sec / 60) % 60),
             static_cast<unsigned long>(sec % 60), static_cast<unsigned long>(boot_count_));

    addHeaders();
    status_ = SdLogStatus::SETUP;
    setup_  = Setup::ROOT;
    Log.printf("INFO  [SdLogger]: SD-Karte gefunden, Ordner %s wird angelegt.\n", session_);
    return true;
}

void SdLogger::addHeaders() {
    for (uint8_t f = 0; f < F_COUNT; f++) {
        const char* h = FILE_HEADERS[f];
        if (!h) continue;
        // Header VOR alles, was schon im Textpuffer liegt.
        const size_t hl = strlen(h);
        if (tlen_[f] + hl > TEXT_BUF) continue;
        memmove(&tbuf_[f][hl], tbuf_[f], tlen_[f]);
        memcpy(tbuf_[f], h, hl);
        tlen_[f] = static_cast<uint16_t>(tlen_[f] + hl);
    }
}

// ---------------------------------------------------------------------------
// Datensaetze in den Ring
// ---------------------------------------------------------------------------

void SdLogger::push(SdRec type, const void* payload, uint8_t len) {
    if (!acceptsData()) return;
    const uint32_t total = REC_HDR + len;
    if (total > 255) return;
    if (BUFFER_SIZE - (head_ - tail_) < total) {
        dropped_++;   // Puffer voll: lieber diesen Datensatz verlieren als blockieren
        return;
    }
    const uint32_t t = millis();
    const uint8_t hdr[REC_HDR] = {
        static_cast<uint8_t>(total), static_cast<uint8_t>(type), state_,
        static_cast<uint8_t>(t), static_cast<uint8_t>(t >> 8),
        static_cast<uint8_t>(t >> 16), static_cast<uint8_t>(t >> 24),
    };
    for (uint8_t k = 0; k < REC_HDR; k++) ring_[(head_ + k) & MASK] = hdr[k];
    const uint8_t* p = static_cast<const uint8_t*>(payload);
    for (uint8_t k = 0; k < len; k++) ring_[(head_ + REC_HDR + k) & MASK] = p[k];
    head_ += total;
}

void SdLogger::pushText(SdRec type, const char* text, size_t len) {
    if (len > 240) len = 240;
    push(type, text, static_cast<uint8_t>(len));
}

void SdLogger::pushLogLine(const char* text, size_t len) {
    if (len == 0) return;
    pushText(SdRec::LOG, text, len);
}

void SdLogger::infof(const char* fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    pushText(SdRec::INFO, buf, static_cast<size_t>(n) < sizeof(buf) ? n : sizeof(buf) - 1);
}

void SdLogger::eventf(const char* name, const char* fmt, ...) {
    char buf[200];
    size_t nl = strlen(name);
    if (nl > 40) nl = 40;
    memcpy(buf, name, nl);
    buf[nl] = '\0';
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(&buf[nl + 1], sizeof(buf) - nl - 1, fmt, ap);
    va_end(ap);
    size_t dl = n < 0 ? 0 : static_cast<size_t>(n);
    if (dl > sizeof(buf) - nl - 2) dl = sizeof(buf) - nl - 2;
    pushText(SdRec::EVT_TEXT, buf, nl + 1 + dl);
}

void SdLogger::pushStateEvent(uint8_t state, uint8_t prev, uint8_t signals, uint8_t prev_signals) {
    const uint8_t p[4] = {state, prev, signals, prev_signals};
    push(SdRec::EVT_STATE, p, sizeof(p));
}

void SdLogger::pushTelecommand(uint8_t opcode, int16_t arg, bool accepted) {
    const uint8_t p[4] = {opcode, static_cast<uint8_t>(accepted ? 1 : 0),
                          static_cast<uint8_t>(arg), static_cast<uint8_t>(arg >> 8)};
    push(SdRec::EVT_TC, p, sizeof(p));
}

void SdLogger::pushSys(SdSys& r) {
    r.sd_fill_pct = fillPercent();
    r.sd_dropped  = dropped_;
    r.sd_kb       = writtenKb();
    push(SdRec::SYS, &r, sizeof(r));
}

void SdLogger::pushArm(uint8_t kind, const uint8_t* frame, uint8_t len) {
    uint8_t p[1 + ARM_MAX_FRAME];
    if (len > ARM_MAX_FRAME) len = ARM_MAX_FRAME;
    p[0] = kind;
    memcpy(&p[1], frame, len);
    push(SdRec::ARM, p, static_cast<uint8_t>(len + 1));
}

uint8_t SdLogger::fillPercent() const {
    return static_cast<uint8_t>(static_cast<uint64_t>(head_ - tail_) * 100 / BUFFER_SIZE);
}

// ---------------------------------------------------------------------------
// Ring -> Text
// ---------------------------------------------------------------------------

bool SdLogger::append(uint8_t file, const char* text, size_t len) {
    if (!room(file, len)) return false;
    memcpy(&tbuf_[file][tlen_[file]], text, len);
    tlen_[file] = static_cast<uint16_t>(tlen_[file] + len);
    return true;
}

void SdLogger::formatPending() {
    bootguard::Scope guard(Stage::LOOP_SD_FORMAT);
    uint8_t rec[256];
    for (uint8_t k = 0; k < FORMAT_PER_CALL && head_ != tail_; k++) {
        const uint8_t len = ring_[tail_ & MASK];
        if (len < REC_HDR || head_ - tail_ < len) {   // darf nicht vorkommen
            tail_ = head_;
            return;
        }
        for (uint8_t i = 0; i < len; i++) rec[i] = ring_[(tail_ + i) & MASK];
        if (!formatRecord(rec, len)) return;   // Zielpuffer voll - spaeter weiter
        tail_ += len;
    }
}

bool SdLogger::formatRecord(const uint8_t* rec, uint8_t len) {
    const SdRec    type  = static_cast<SdRec>(rec[1]);
    const uint8_t  state = rec[2];
    const uint32_t t     = rdU32(&rec[3]);
    const uint8_t* p     = &rec[REC_HDR];
    const uint8_t  plen  = static_cast<uint8_t>(len - REC_HDR);
    Line l;

    switch (type) {
        case SdRec::LOG: {
            l.c('[');
            l.fix(t, 3);
            l.s("] ");
            l.s(reinterpret_cast<const char*>(p), plen);
            l.nl();
            // Warnungen zusaetzlich in die Zeitleiste.
            const bool warn = plen >= 4 && (memcmp(p, "WARN", 4) == 0 || memcmp(p, "ERR", 3) == 0);
            if (!warn) return append(F_LOG, l.b, l.n);
            Line e;
            rowStart(e, t, state);
            e.s("WARNUNG,");
            e.quoted(reinterpret_cast<const char*>(p), plen);
            e.nl();
            if (!room(F_LOG, l.n) || !room(F_EVENTS, e.n)) return false;
            append(F_LOG, l.b, l.n);
            append(F_EVENTS, e.b, e.n);
            return true;
        }

        case SdRec::INFO:
            l.s(reinterpret_cast<const char*>(p), plen);
            l.s("\r\n");
            return append(F_INFO, l.b, l.n);

        case SdRec::EVT_STATE: {
            rowStart(l, t, state);
            const bool changed = p[0] != p[1];
            l.s(changed ? "ZUSTAND," : "SIGNAL,");
            l.c('"');
            if (changed) { l.s(stateName(p[1])); l.s(" -> "); l.s(stateName(p[0])); l.s(" | "); }
            signalText(l, p[2]);
            l.s(" (vorher ");
            signalText(l, p[3]);
            l.s(")\"");
            l.nl();
            return append(F_EVENTS, l.b, l.n);
        }

        case SdRec::EVT_TC: {
            rowStart(l, t, state);
            l.s("TELECOMMAND,\"0x");
            l.hex(p[0], 2);
            l.c(' ');
            l.s(opcodeName(p[0]));
            l.s(" arg=");
            l.i(static_cast<int16_t>(p[2] | (p[3] << 8)));
            l.s(p[1] ? " angenommen\"" : " ABGEWIESEN (Zustand)\"");
            l.nl();
            return append(F_EVENTS, l.b, l.n);
        }

        case SdRec::EVT_TEXT: {
            rowStart(l, t, state);
            const size_t nl = strnlen(reinterpret_cast<const char*>(p), plen);
            l.s(reinterpret_cast<const char*>(p), nl);
            l.comma();
            const size_t off = nl + 1 < plen ? nl + 1 : plen;
            l.quoted(reinterpret_cast<const char*>(p + off), plen - off);
            l.nl();
            return append(F_EVENTS, l.b, l.n);
        }

        case SdRec::IMU: {
            SdImu r;
            memcpy(&r, p, sizeof(r));
            rowStart(l, t, state);
            // +-3 g: 10920 Counts/g; +-500 Grad/s: 65,536 Counts/(Grad/s)
            for (uint8_t k = 0; k < 3; k++) { l.fix(scaled(r.acc[k], 100000, 10920), 5); l.comma(); }
            for (uint8_t k = 0; k < 3; k++) { l.fix(scaled(r.gyro[k], 1000000, 65536), 3); l.comma(); }
            l.fix(static_cast<int64_t>(r.temp) * 125 + 23000, 3);   // 0,125 K/LSB + 23
            l.comma();
            l.flag(r.valid);
            l.nl();
            return append(F_IMU, l.b, l.n);
        }

        case SdRec::FORCE: {
            SdForce r;
            memcpy(&r, p, sizeof(r));
            const uint8_t n = r.count > 4 ? 4 : r.count;
            rowStart(l, t, state);
            for (uint8_t k = 0; k < n; k++) { l.i(r.counts[k]); l.comma(); }
            for (uint8_t k = 0; k < n; k++) { l.i(r.raw[k]); l.comma(); }
            l.flag(r.flags & SD_FORCE_TARED); l.comma();
            l.flag(r.flags & SD_FORCE_STALE); l.comma();
            l.flag(r.flags & SD_FORCE_FRESH);
            l.nl();
            return append(r.sensor == 2 ? F_FORCE_T2 : F_FORCE_T1, l.b, l.n);
        }

        case SdRec::MOTOR: {
            SdMotor r;
            memcpy(&r, p, sizeof(r));
            rowStart(l, t, state);
            l.u(r.motor); l.comma();
            l.i(r.position); l.comma();
            l.i(r.pwm); l.comma();
            l.flag(r.state & DL_MOTOR_STATE_ON); l.comma();
            l.flag(r.state & DL_MOTOR_STATE_ENCODER_OK); l.comma();
            l.flag(r.state & DL_MOTOR_STATE_TURNING); l.comma();
            l.flag(r.state & DL_MOTOR_STATE_TURN_FAILED);
            l.nl();
            return append(F_MOTOR, l.b, l.n);
        }

        case SdRec::CAM: {
            SdCam r;
            memcpy(&r, p, sizeof(r));
            rowStart(l, t, state);
            l.u(r.cam); l.comma();
            l.flag(r.flags & DL_CAM_ALIVE); l.comma();
            if (r.age_s != 255) l.u(r.age_s);
            l.comma();
            l.u(r.protocol); l.comma();
            l.hex(r.features, 4); l.comma();
            l.flag(r.flags & DL_CAM_FEAT_START); l.comma();
            l.flag(r.flags & DL_CAM_FEAT_STOP); l.comma();
            l.s((r.flags & DL_CAM_SENT_START) ? "START" : (r.flags & DL_CAM_SENT_STOP) ? "STOP" : "-");
            l.comma();
            l.u(r.probes_failed); l.comma();
            l.flag(r.flags & DL_CAM_RX_ANY); l.comma();
            l.flag(r.flags & DL_CAM_RX_GARBAGE);
            l.nl();
            return append(F_CAMERA, l.b, l.n);
        }

        case SdRec::SYS: {
            SdSys r;
            memcpy(&r, p, sizeof(r));
            rowStart(l, t, state);
            const uint32_t sec = r.rtc % 86400;
            l.c(static_cast<char>('0' + sec / 36000)); l.c(static_cast<char>('0' + (sec / 3600) % 10));
            l.c(':');
            l.c(static_cast<char>('0' + (sec / 600) % 6)); l.c(static_cast<char>('0' + (sec / 60) % 10));
            l.c(':');
            l.c(static_cast<char>('0' + (sec % 60) / 10)); l.c(static_cast<char>('0' + sec % 10));
            l.comma();
            l.hex(r.subsys, 2); l.comma();
            l.flag(r.healthy); l.comma();
            l.hex(r.rexus_raw, 2); l.comma();
            l.u(r.uplink_rx); l.comma();
            l.u(r.uplink_ok); l.comma();
            l.u(r.uplink_bad); l.comma();
            l.hex(r.uplink_last_op, 2); l.comma();
            l.flag(r.arm_alive); l.comma();
            l.u(r.arm_ok); l.comma();
            l.u(r.arm_crc_errors + r.arm_unknown_ids); l.comma();
            l.hex(r.arm_last_msg, 2); l.comma();
            l.u(r.arm_last_node); l.comma();
            l.u(r.arm_crc_errors); l.comma();
            l.u(r.arm_unknown_ids); l.comma();
            l.u(r.arm_freefall); l.comma();
            l.u(r.arm_tx_dropped); l.comma();
            l.u(r.traj_phase); l.comma();
            l.u(r.flight_phase); l.comma();
            l.hex(r.flight_flags, 2); l.comma();
            l.flag(r.flight_switch); l.comma();
            l.u(r.cams_alive); l.comma();
            l.u(r.dl_dropped_status); l.comma();
            l.u(r.dl_dropped_motion); l.comma();
            l.u(r.dl_dropped_bulk); l.comma();
            l.u(r.sd_fill_pct); l.comma();
            l.u(r.sd_dropped); l.comma();
            l.u(r.sd_kb); l.comma();
            l.fix(r.loop_max_us / 10, 2); l.comma();   // us -> ms, 2 Nachkommastellen
            l.u(r.loops);
            l.nl();
            return append(F_SYS, l.b, l.n);
        }

        case SdRec::ARM: {
            if (plen < 1 + ARM_HEADER_SIZE) return true;   // kaputt - ueberspringen
            const ArmTapKind kind = static_cast<ArmTapKind>(p[0]);
            const bool     tx   = kind == ArmTapKind::TX;
            const bool     bad  = kind == ArmTapKind::RX_BAD_CRC || kind == ArmTapKind::RX_BAD_ID;
            const uint8_t  node = p[2];
            const uint8_t  id   = p[3];
            const uint8_t* d    = &p[1 + ARM_HEADER_SIZE];
            const uint8_t  dlen = static_cast<uint8_t>(plen - 1 - ARM_HEADER_SIZE);

            // Rohzeile - immer.
            rowStart(l, t, state);
            l.s(tx ? "TX," : kind == ArmTapKind::RX_BAD_CRC ? "RX_CRC_FEHLER,"
                  : kind == ArmTapKind::RX_BAD_ID ? "RX_ID_UNBEKANNT," : "RX,");
            l.u(node); l.comma();
            l.hex(id, 2); l.comma();
            l.s(armMsgName(id)); l.comma();
            for (uint8_t k = 0; k < dlen; k++) l.hex(d[k], 2);
            l.nl();

            // Ausgewertete Zeile (nur Empfang).
            Line v;
            uint8_t vfile = F_COUNT;
            if (!tx && !bad) {
                const ArmMsg m = static_cast<ArmMsg>(id);
                if (m == ArmMsg::JOINT_STATE && dlen >= 5) {
                    const int32_t urad = static_cast<int32_t>(rdU32(&d[1]));
                    rowStart(v, t, state);
                    v.u(node); v.comma();
                    v.hex(d[0], 2); v.comma();
                    // urad -> Grad*1e4: urad * 1,8e7 / (pi * 1e7)
                    v.fix(scaled(urad, 18000000LL, 31415927LL), 4); v.comma();
                    v.i(urad);
                    v.nl();
                    vfile = F_ARM_JOINTS;
                } else if (m == ArmMsg::IMU_DATA && dlen >= 14) {
                    rowStart(v, t, state);
                    v.u(node); v.comma();
                    v.hex(d[0], 2); v.comma();
                    v.hex(d[1], 2);
                    for (uint8_t k = 0; k < 6; k++) { v.comma(); v.i(rdI16(&d[2 + 2 * k])); }
                    v.nl();
                    vfile = F_ARM_MODULE_IMU;
                } else if (m == ArmMsg::VOLTAGE_DATA && dlen >= 4) {
                    rowStart(v, t, state);
                    v.u(node); v.comma();
                    v.fix(rdU16(&d[0]), 3); v.comma();
                    v.fix(rdU16(&d[2]), 3); v.s(",,");
                    v.nl();
                    vfile = F_ARM_POWER;
                } else if (m == ArmMsg::TEMPERATURE_DATA && dlen >= 4) {
                    rowStart(v, t, state);
                    v.u(node); v.s(",,,");
                    v.fix(rdI16(&d[0]), 2); v.comma();   // /100 wie Server (ARM_TEMP_DIV)
                    v.fix(rdI16(&d[2]), 2);
                    v.nl();
                    vfile = F_ARM_POWER;
                } else if (m == ArmMsg::RCU_IMU_DATA && dlen >= 12) {
                    rowStart(v, t, state);
                    // +-4 g: 8192 Counts/g; +-500 Grad/s: 65,536 Counts/(Grad/s)
                    for (uint8_t k = 0; k < 3; k++) { v.fix(scaled(rdI16(&d[2 * k]), 100000, 8192), 5); v.comma(); }
                    for (uint8_t k = 0; k < 3; k++) {
                        v.fix(scaled(rdI16(&d[6 + 2 * k]), 1000000, 65536), 3);
                        if (k < 2) v.comma();
                    }
                    v.nl();
                    vfile = F_ARM_RCU_IMU;
                } else if (m == ArmMsg::REFERENCE_ANGLE_DATA || m == ArmMsg::IDENTIFIED ||
                           m == ArmMsg::RCU_FREEFALL_EVENT) {
                    rowStart(v, t, state);
                    v.s("ARM_"); v.s(armMsgName(id)); v.s(",\"Modul ");
                    v.u(node);
                    if (m == ArmMsg::REFERENCE_ANGLE_DATA && dlen >= 4) {
                        v.s(" Winkel ");
                        v.fix(scaled(static_cast<int32_t>(rdU32(d)), 18000000LL, 31415927LL), 4);
                        v.s(" Grad");
                    } else if (dlen) {
                        v.s(" Daten ");
                        for (uint8_t k = 0; k < dlen; k++) v.hex(d[k], 2);
                    }
                    v.c('"');
                    v.nl();
                    vfile = F_EVENTS;
                }
            }
            if (!room(F_ARM_RAW, l.n) || (vfile < F_COUNT && !room(vfile, v.n))) return false;
            append(F_ARM_RAW, l.b, l.n);
            if (vfile < F_COUNT) append(vfile, v.b, v.n);
            return true;
        }
    }
    return true;   // unbekannter Typ - ueberspringen
}

// ---------------------------------------------------------------------------
// Karte bedienen
// ---------------------------------------------------------------------------

void SdLogger::service(uint32_t now_ms, bool io_allowed) {
    if (status_ == SdLogStatus::NO_CARD || status_ == SdLogStatus::FAILED ||
        status_ == SdLogStatus::DISABLED) {
        head_ = tail_ = 0;
        return;
    }
    if (status_ == SdLogStatus::BUFFERING) return;   // begin() noch nicht gelaufen

    formatPending();
    if (!io_allowed) return;

    bootguard::Scope guard(Stage::LOOP_SD);
    if (setup_ != Setup::RUN) setupStep(now_ms);
    else                      runStep(now_ms);
}

bool SdLogger::checkDuration(uint32_t start_us, uint32_t now_ms, const char* what) {
    const uint32_t dt = micros() - start_us;
    if (dt <= SLOW_US) return true;
    if (dt > HARD_SLOW_US) {
        Log.printf("WARN  [SdLogger]: %s dauerte %lu ms.\n", what, static_cast<unsigned long>(dt / 1000));
        fail(SdLogFail::SLOW, what);
        return false;
    }
    if (slow_count_ == 0 || now_ms - slow_first_ms_ > SLOW_WINDOW_MS) {
        slow_count_ = 1;
        slow_first_ms_ = now_ms;
    } else {
        slow_count_++;
    }
    Log.printf("WARN  [SdLogger]: %s langsam (%lu ms, %u. Mal).\n", what,
               static_cast<unsigned long>(dt / 1000), slow_count_);
    if (slow_count_ >= SLOW_LIMIT) {
        fail(SdLogFail::SLOW, what);
        return false;
    }
    return true;
}

static bool ensureDir(const char* path) {
    if (SD.exists(path)) return true;
    return SD.mkdir(path);
}

void SdLogger::setupStep(uint32_t now_ms) {
    const uint32_t t0 = micros();
    switch (setup_) {
        case Setup::ROOT:
            if (!ensureDir("/MAGGIE")) { fail(SdLogFail::MKDIR, "Ordner /MAGGIE"); return; }
            if (checkDuration(t0, now_ms, "Ordner /MAGGIE")) setup_ = Setup::README;
            return;

        case Setup::README: {
            static const char* const path = "/MAGGIE/LIESMICH.TXT";
            if (!SD.exists(path)) {
                File f = SD.open(path, FILE_WRITE_BEGIN);
                if (!f) { fail(SdLogFail::README, "LIESMICH.TXT"); return; }
                const size_t n = sizeof(README_TEXT) - 1;
                const size_t w = f.write(reinterpret_cast<const uint8_t*>(README_TEXT), n);
                f.close();
                if (w != n) { fail(SdLogFail::README, "LIESMICH.TXT"); return; }
            }
            if (checkDuration(t0, now_ms, "LIESMICH.TXT")) setup_ = Setup::DATE;
            return;
        }

        case Setup::DATE:
            if (!ensureDir(date_dir_)) { fail(SdLogFail::MKDIR, "Tagesordner"); return; }
            if (checkDuration(t0, now_ms, "Tagesordner")) setup_ = Setup::SESSION;
            return;

        case Setup::SESSION: {
            // Gibt es den Namen schon (gleiche Sekunde, Boot-Zaehler
            // zurueckgesetzt), wird ein Suffix angehaengt - nie ueberschreiben.
            const size_t base = strlen(session_);
            bool ok = false;
            for (uint8_t suffix = 0; suffix < 10 && !ok; suffix++) {
                if (suffix) snprintf(&session_[base], sizeof(session_) - base, "_%u", suffix);
                ok = !SD.exists(session_) && SD.mkdir(session_);
                if (!ok) session_[base] = '\0';
            }
            if (!ok) { fail(SdLogFail::MKDIR, "Startordner"); return; }
            if (checkDuration(t0, now_ms, "Startordner")) { setup_ = Setup::OPEN; open_next_ = 0; }
            return;
        }

        case Setup::OPEN: {
            char path[72];
            snprintf(path, sizeof(path), "%s/%s", session_, FILE_NAMES[open_next_]);
            files_[open_next_] = SD.open(path, FILE_WRITE_BEGIN);
            if (!files_[open_next_]) { fail(SdLogFail::OPEN, FILE_NAMES[open_next_]); return; }
            last_sync_[open_next_] = now_ms;
            if (!checkDuration(t0, now_ms, FILE_NAMES[open_next_])) return;
            if (++open_next_ >= F_COUNT) {
                setup_  = Setup::RUN;
                status_ = SdLogStatus::ACTIVE;
                Log.printf("INFO  [SdLogger]: Bordspeicher aktiv: %s\n", session_);
            }
            return;
        }

        case Setup::RUN:
            return;
    }
}

bool SdLogger::writeFile(uint8_t file, uint32_t n, uint32_t now_ms) {
    const uint32_t t0 = micros();
    const size_t w = files_[file].write(reinterpret_cast<const uint8_t*>(tbuf_[file]), n);
    if (w != n) { fail(SdLogFail::WRITE, FILE_NAMES[file]); return false; }
    if (!checkDuration(t0, now_ms, FILE_NAMES[file])) return false;
    tlen_[file] = static_cast<uint16_t>(tlen_[file] - n);
    memmove(tbuf_[file], &tbuf_[file][n], tlen_[file]);
    dirty_[file] = true;
    written_ += n;
    return true;
}

void SdLogger::runStep(uint32_t now_ms) {
    // 1. Ein voller Sektor - von der vollsten Datei.
    int best = -1;
    uint16_t best_len = CHUNK_SIZE - 1;
    for (uint8_t f = 0; f < F_COUNT; f++) {
        if (tlen_[f] > best_len) { best = f; best_len = tlen_[f]; }
    }
    if (best >= 0) {
        writeFile(static_cast<uint8_t>(best), CHUNK_SIZE, now_ms);
        return;
    }

    // 2. Reihum sichern: Rest schreiben, im naechsten Aufruf sync().
    for (uint8_t k = 0; k < F_COUNT; k++) {
        const uint8_t f = static_cast<uint8_t>((sync_rr_ + k) % F_COUNT);
        if (!(tlen_[f] > 0 || dirty_[f]) || now_ms - last_sync_[f] < SYNC_MS) continue;
        if (tlen_[f] > 0) {
            writeFile(f, tlen_[f], now_ms);
            sync_rr_ = f;
            return;
        }
        const uint32_t t0 = micros();
        files_[f].flush();
        if (files_[f].getWriteError()) { fail(SdLogFail::SYNC, FILE_NAMES[f]); return; }
        if (!checkDuration(t0, now_ms, FILE_NAMES[f])) return;
        dirty_[f] = false;
        last_sync_[f] = now_ms;
        sync_rr_ = static_cast<uint8_t>((f + 1) % F_COUNT);
        return;
    }
}

void SdLogger::fail(SdLogFail why, const char* what) {
    // Die Karte NICHT mehr anfassen - auch kein close(): Bei einer defekten
    // Karte wuerde genau das wieder lange haengen.
    status_ = SdLogStatus::FAILED;
    fail_   = why;
    head_ = tail_ = 0;
    Log.printf("WARN  [SdLogger]: SD-Fehler (%s, Grund %u) - Bordspeicher fuer diesen Start aus.\n",
               what, static_cast<unsigned>(why));
}
