// Host-Test des SD-Loggers (src/storage/sd_logger.cpp).
//
// Die "Karte" ist ein Ordner auf dem PC (siehe SD.h). Geprueft wird:
//  - Ordnerstruktur, Dateinamen, Kopfzeilen
//  - jede CSV-Zeile hat so viele Felder wie die Kopfzeile
//  - Umrechnungen (g, Grad/s, Grad C, Grad, V) an bekannten Werten
//  - kein Datensatz geht verloren, auch nicht ueber den Ring-Ueberlauf
//    (256 KB) hinweg und nicht waehrend einer Motorfahrt (keine Kartenzugriffe)
//  - langsame Karte, Schreibfehler, fehlende Karte -> aus, Karte bleibt unberuehrt
//  - voller Puffer -> Datensaetze werden gezaehlt verworfen, CSV bleibt gueltig
//
//   tools/sd_sim/run.sh                 # alle Szenarien
//   SD_SIM_KEEP=1 tools/sd_sim/run.sh   # Kartenordner danach behalten (Pfad wird ausgegeben)
#include <Arduino.h>
#include <SD.h>
#include "obc_log.hpp"
#include "boot_guard.hpp"
#include "storage/sd_logger.hpp"
#include "hal/arm_hal.hpp"
#include "hal/uplink_hal.hpp"
#include "statemachine/mission_state.hpp"

#include <sys/stat.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <dirent.h>

// ---------------------------------------------------------------------------
// Stubs
// ---------------------------------------------------------------------------
static uint64_t g_us = 0;
uint32_t millis() { return static_cast<uint32_t>(g_us / 1000); }
uint32_t micros() { return static_cast<uint32_t>(g_us); }
void fakeAdvanceUs(uint32_t us) { g_us += us; }
static unsigned long g_rtc = 1795081512;   // 2026-11-19 09:45:12 UTC
unsigned long rtc_get() { return g_rtc + static_cast<unsigned long>(g_us / 1000000); }

FakeLog Log;
FakeCard g_card;
SDClass SD;

static std::string hostPath(const char* p) { return g_card.root + p; }
bool SDClass::exists(const char* path) {
    g_card.ops++;
    fakeAdvanceUs(g_card.op_delay_us);
    struct stat st;
    return stat(hostPath(path).c_str(), &st) == 0;
}
bool SDClass::mkdir(const char* path) {
    g_card.ops++;
    fakeAdvanceUs(g_card.op_delay_us);
    return ::mkdir(hostPath(path).c_str(), 0755) == 0;
}
File SDClass::open(const char* path, int) {
    g_card.ops++;
    fakeAdvanceUs(g_card.op_delay_us);
    return File(fopen(hostPath(path).c_str(), "wb"));
}

namespace bootguard {
static Stage s_stage = Stage::NONE;
void stage(Stage s) { s_stage = s; }
Stage currentStage() { return s_stage; }
void watchdogFeed() {}
}

// ---------------------------------------------------------------------------
// Pruefhilfen
// ---------------------------------------------------------------------------
static int g_failed = 0;
static void check(bool ok, const std::string& what) {
    printf("    [%s] %s\n", ok ? " OK " : "FEHL", what.c_str());
    if (!ok) g_failed++;
}

static std::vector<std::string> readLines(const std::string& path) {
    std::vector<std::string> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        out.push_back(line);
    }
    return out;
}

static size_t fieldCount(const std::string& line) {
    size_t n = 1;
    bool q = false;
    for (char c : line) {
        if (c == '"') q = !q;
        else if (c == ',' && !q) n++;
    }
    return n;
}

/// Prueft eine CSV-Datei: alle Zeilen so viele Felder wie der Kopf. Rueckgabe: Datenzeilen.
static long csvRows(const std::string& path, const std::string& name) {
    const auto lines = readLines(path);
    if (lines.empty()) { check(false, name + " existiert und hat einen Kopf"); return -1; }
    const size_t want = fieldCount(lines[0]);
    size_t bad = 0;
    for (size_t i = 1; i < lines.size(); i++) if (fieldCount(lines[i]) != want) bad++;
    check(bad == 0, name + ": alle " + std::to_string(lines.size() - 1) + " Zeilen mit " +
                    std::to_string(want) + " Feldern" + (bad ? " (" + std::to_string(bad) + " falsch)" : ""));
    return static_cast<long>(lines.size()) - 1;
}

static bool contains(const std::string& path, const std::string& needle) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str().find(needle) != std::string::npos;
}

static std::string newCard(const char* name) {
    char tmpl[256];
    snprintf(tmpl, sizeof(tmpl), "%s/maggie_sd_sim_%s_XXXXXX", getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp", name);
    const char* d = mkdtemp(tmpl);
    g_card = FakeCard{};
    g_card.root = d;
    return d;
}

// ---------------------------------------------------------------------------
// Datenquellen wie im OBC
// ---------------------------------------------------------------------------
static void armFrame(SdLogger& sd, bool tx, uint8_t node, ArmMsg id, const uint8_t* p, uint8_t n) {
    uint8_t f[ARM_HEADER_SIZE + ARM_MAX_PAYLOAD];
    f[0] = ARM_SYNC; f[1] = node; f[2] = static_cast<uint8_t>(id); f[3] = 0;
    memcpy(&f[4], p, n);
    sd.pushArm(static_cast<uint8_t>(tx ? ArmTapKind::TX : ArmTapKind::RX), f,
               static_cast<uint8_t>(ARM_HEADER_SIZE + n));
}
static void le16(uint8_t* p, int16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void le32(uint8_t* p, int32_t v) { for (int k = 0; k < 4; k++) p[k] = (v >> (8 * k)) & 0xFF; }

struct Counts {
    long imu = 0, force1 = 0, force2 = 0, motor = 0, sys = 0, cam = 0;
    long joints = 0, module_imu = 0, power = 0, rcu_imu = 0, arm_raw = 0, events = 0;
};

/// Ein Loop-Tick (1 ms) mit allen Quellen.
static void tick(SdLogger& sd, uint32_t ms, Counts& c, bool io_allowed) {
    if (g_us < static_cast<uint64_t>(ms) * 1000) g_us = static_cast<uint64_t>(ms) * 1000;
    if (ms % 10 == 0) {
        SdImu r{};
        r.acc[0] = 10920; r.acc[1] = -5460; r.acc[2] = 0;
        r.gyro[0] = 6554; r.gyro[1] = 0; r.gyro[2] = -6554;
        r.temp = 8; r.valid = 1;
        sd.pushImu(r); c.imu++;
        SdMotor m{1, 0x03, -120, static_cast<int32_t>(ms)};
        sd.pushMotor(m); m.motor = 2; sd.pushMotor(m); c.motor += 2;
        uint8_t p[12];
        for (int k = 0; k < 3; k++) le16(&p[2 * k], k == 2 ? 8192 : 0);
        for (int k = 0; k < 3; k++) le16(&p[6 + 2 * k], 0);
        armFrame(sd, false, 0, ArmMsg::RCU_IMU_DATA, p, 12); c.rcu_imu++; c.arm_raw++;
    }
    if (ms % 12 == 0) {
        SdForce f{}; f.sensor = 1; f.count = 3; f.flags = SD_FORCE_TARED | SD_FORCE_FRESH;
        for (int k = 0; k < 3; k++) { f.counts[k] = -1000 * k; f.raw[k] = 123456 + k; }
        sd.pushForce(f); c.force1++;
    }
    if (ms % 13 == 0) {
        SdForce f{}; f.sensor = 2; f.count = 4; f.flags = SD_FORCE_TARED | SD_FORCE_FRESH;
        for (int k = 0; k < 4; k++) { f.counts[k] = 7 * k; f.raw[k] = -8388608 + k; }
        sd.pushForce(f); c.force2++;
    }
    if (ms % 100 == 0) {
        for (uint8_t n = 1; n <= 7; n++) {
            uint8_t p[5]; p[0] = 0x21; le32(&p[1], static_cast<int32_t>(17453.2925 * n + 0.5));   // n Grad
            armFrame(sd, false, n, ArmMsg::JOINT_STATE, p, 5); c.joints++; c.arm_raw++;
            uint8_t t[4]; le32(t, 17453 * n);
            armFrame(sd, true, n, ArmMsg::STEPPER_MOVE_ABS, t, 4); c.arm_raw++;
        }
    }
    if (ms % 50 == 0) {
        uint8_t p[16] = {0x48, 0x4C};
        for (int k = 0; k < 6; k++) le16(&p[2 + 2 * k], static_cast<int16_t>(100 * (k + 1)));
        armFrame(sd, false, 1, ArmMsg::IMU_DATA, p, 16); c.module_imu++; c.arm_raw++;
    }
    if (ms % 1000 == 500) {
        for (uint8_t n = 1; n <= 7; n++) {
            uint8_t v[4]; le16(v, 24000); le16(&v[2], 12000);
            armFrame(sd, false, n, ArmMsg::VOLTAGE_DATA, v, 4); c.power++; c.arm_raw++;
            uint8_t t[4]; le16(t, 3150); le16(&t[2], -250);
            armFrame(sd, false, n, ArmMsg::TEMPERATURE_DATA, t, 4); c.power++; c.arm_raw++;
        }
    }
    if (ms % 1000 == 0) {
        SdSys s{}; s.rtc = static_cast<uint32_t>(rtc_get()); s.subsys = 0x3F; s.healthy = 1;
        s.loop_max_us = 1234; s.loops = 5000;
        sd.pushSys(s); c.sys++;
        for (uint8_t k = 1; k <= 4; k++) {
            SdCam cam{k, 0x17, static_cast<uint8_t>(k == 4 ? 255 : 1), 1, 0x0077, 3};
            sd.pushCam(cam); c.cam++;
        }
        char line[80];
        const int n = snprintf(line, sizeof(line), "INFO  [Sim]: Sekunde %u, Text mit, Komma \"zitiert\"", ms / 1000);
        sd.pushLogLine(line, static_cast<size_t>(n));
    }
    if (ms == 2000) {
        sd.pushTelecommand(static_cast<uint8_t>(UplinkOpcode::TEST_ENTER), 0, true); c.events++;
        sd.setState(static_cast<uint8_t>(MissionState::TEST));
        sd.pushStateEvent(static_cast<uint8_t>(MissionState::TEST),
                          static_cast<uint8_t>(MissionState::PRE_LAUNCH), 0x01, 0x00); c.events++;
        sd.pushTelecommand(static_cast<uint8_t>(UplinkOpcode::MOTOR_ON), -200, false); c.events++;
    }
    if (ms == 3000) {
        const char* w = "WARN  [Sim]: Testwarnung, mit Komma";
        sd.pushLogLine(w, strlen(w)); c.events++;
        sd.eventf("TEST_EREIGNIS", "Wert %d, noch einer %d", 42, 7); c.events++;
    }
    if (ms == 5000) {
        armFrame(sd, false, 0, ArmMsg::RCU_FREEFALL_EVENT, nullptr, 0); c.events++; c.arm_raw++;
        uint8_t p[4]; le32(p, 1570796);   // 90 Grad
        armFrame(sd, false, 3, ArmMsg::REFERENCE_ANGLE_DATA, p, 4); c.events++; c.arm_raw++;
    }
    if (g_us < static_cast<uint64_t>(ms) * 1000) g_us = static_cast<uint64_t>(ms) * 1000;
    sd.service(ms, io_allowed);
}

static std::string findSession(const std::string& root) {
    const std::string day = root + "/MAGGIE/2026-11-19";
    DIR* d = opendir(day.c_str());
    if (!d) return "";
    std::string found;
    while (dirent* e = readdir(d)) {
        if (e->d_name[0] != '.') { found = day + "/" + e->d_name; }
    }
    closedir(d);
    return found;
}

// ---------------------------------------------------------------------------
// Szenarien
// ---------------------------------------------------------------------------
static void scenarioNormal() {
    printf("\n== Normalbetrieb: 40 s, alle Quellen, Motorfahrt 8-12 s ==\n");
    const std::string root = newCard("normal");
    g_us = 0;
    static SdLogger sd;
    sd = SdLogger{};
    { const char* b = "INFO  [System]: Boot-Meldung vor begin()"; sd.pushLogLine(b, strlen(b)); }
    sd.infof("MAGGIE OBC - Start Nr. %u", 42);
    check(sd.begin(42, false), "begin() findet die Karte");
    Counts c;
    uint32_t max_fill = 0;
    for (uint32_t ms = 1; ms <= 40000; ms++) {
        const bool motor = ms >= 8000 && ms < 12000;
        const uint32_t ops_before = g_card.ops;
        tick(sd, ms, c, !motor);
        if (motor && g_card.ops != ops_before) { check(false, "Kartenzugriff waehrend Motorfahrt"); break; }
        if (sd.fillPercent() > max_fill) max_fill = sd.fillPercent();
    }
    // Nachlauf ohne neue Daten: alles schreiben und sichern.
    for (uint32_t ms = 40001; ms <= 46000; ms++) { g_us = ms * 1000ULL; sd.service(ms, true); }

    check(sd.status() == SdLogStatus::ACTIVE, "Status ACTIVE");
    check(sd.dropped() == 0, "kein Datensatz verworfen (max. Puffer " + std::to_string(max_fill) + " %)");
    printf("    geschrieben: %u KB, Kartenoperationen: %u\n", sd.writtenKb(), g_card.ops);
    check(sd.writtenKb() > 256, "mehr als ein Ringdurchlauf (256 KB) geschrieben");

    const std::string s = findSession(root);
    check(s.size() && s.find("094512_B00042") != std::string::npos, "Startordner " + s.substr(root.size()));
    struct stat st;
    check(stat((root + "/MAGGIE/LIESMICH.TXT").c_str(), &st) == 0 && st.st_size > 1000, "LIESMICH.TXT angelegt");

    check(csvRows(s + "/IMU.csv", "IMU.csv") == c.imu, "IMU: alle " + std::to_string(c.imu) + " Zeilen");
    check(csvRows(s + "/FORCE_T1.csv", "FORCE_T1.csv") == c.force1, "FORCE_T1: alle Zeilen");
    check(csvRows(s + "/FORCE_T2.csv", "FORCE_T2.csv") == c.force2, "FORCE_T2: alle Zeilen");
    check(csvRows(s + "/MOTOR.csv", "MOTOR.csv") == c.motor, "MOTOR: alle Zeilen");
    check(csvRows(s + "/SYS.csv", "SYS.csv") == c.sys, "SYS: alle Zeilen");
    check(csvRows(s + "/CAMERA.csv", "CAMERA.csv") == c.cam, "CAMERA: alle Zeilen");
    check(csvRows(s + "/ARM_JOINTS.csv", "ARM_JOINTS.csv") == c.joints, "ARM_JOINTS: alle Zeilen");
    check(csvRows(s + "/ARM_MODULE_IMU.csv", "ARM_MODULE_IMU.csv") == c.module_imu, "ARM_MODULE_IMU: alle Zeilen");
    check(csvRows(s + "/ARM_POWER.csv", "ARM_POWER.csv") == c.power, "ARM_POWER: alle Zeilen");
    check(csvRows(s + "/ARM_RCU_IMU.csv", "ARM_RCU_IMU.csv") == c.rcu_imu, "ARM_RCU_IMU: alle Zeilen");
    check(csvRows(s + "/ARM_RAW.csv", "ARM_RAW.csv") == c.arm_raw, "ARM_RAW: alle Zeilen (RX + TX)");
    check(csvRows(s + "/EVENTS.csv", "EVENTS.csv") == c.events, "EVENTS: alle " + std::to_string(c.events) + " Ereignisse");

    check(contains(s + "/IMU.csv", ",PRE_LAUNCH,1.00000,-0.50000,0.00000,100.006,0.000,-100.006,24.000,1"),
          "IMU umgerechnet: 1 g / -0,5 g / 100 Grad/s / 24 Grad C");
    check(contains(s + "/IMU.csv", ",TEST,1.00000"), "IMU: Zustandsspalte wechselt auf TEST");
    check(contains(s + "/ARM_JOINTS.csv", ",7,21,7.0000,122173"), "Gelenk 7: 7.0000 Grad");
    check(contains(s + "/ARM_POWER.csv", ",3,24.000,12.000,,") && contains(s + "/ARM_POWER.csv", ",3,,,31.50,-2.50"),
          "Spannung 24 V / 12 V, Temperatur 31,5 / -2,5 Grad C");
    check(contains(s + "/ARM_RCU_IMU.csv", ",0.00000,0.00000,1.00000,0.000,0.000,0.000"), "RCU-IMU 1 g in Z");
    check(contains(s + "/ARM_MODULE_IMU.csv", ",1,48,4C,100,200,300,400,500,600"), "Modul-IMU roh");
    check(contains(s + "/ARM_RAW.csv", ",TX,7,07,MOVE_ABS,"), "gesendete Arm-Frames in ARM_RAW");
    check(contains(s + "/FORCE_T2.csv", ",0,7,14,21,-8388608,-8388607,-8388606,-8388605,1,0,1"), "Kraft 2: tariert + roh");
    check(contains(s + "/EVENTS.csv", "ZUSTAND,\"PRE_LAUNCH -> TEST | LO=1 SOE=0 SODS=0"), "Zustandswechsel");
    check(contains(s + "/EVENTS.csv", "TELECOMMAND,\"0x01 MOTOR_ON arg=-200 ABGEWIESEN"), "abgewiesenes Telecommand");
    check(contains(s + "/EVENTS.csv", "WARNUNG,\"WARN  [Sim]: Testwarnung, mit Komma\""), "Warnung in der Zeitleiste");
    check(contains(s + "/EVENTS.csv", "TEST_EREIGNIS,\"Wert 42, noch einer 7\""), "freies Ereignis");
    check(contains(s + "/EVENTS.csv", "ARM_REFERENCE_ANGLE,\"Modul 3 Winkel 90.0000 Grad\""), "Referenzwinkel");
    check(contains(s + "/EVENTS.csv", "ARM_RCU_FREEFALL"), "Freefall-Ereignis");
    check(contains(s + "/LOG.txt", "] INFO  [System]: Boot-Meldung vor begin()"), "Boot-Meldung vor begin() im LOG");
    check(contains(s + "/LOG.txt", "Text mit, Komma \"zitiert\""), "LOG unveraendert");
    check(contains(s + "/INFO.txt", "MAGGIE OBC - Start Nr. 42"), "INFO.txt");
    check(contains(s + "/SYS.csv", ",09:45:3"), "SYS: Uhrzeit");
    check(contains(s + "/SYS.csv", ",1.23,5000"), "SYS: Loop-Dauer 1234 us = 1.23 ms");
    check(contains(s + "/CAMERA.csv", ",4,1,,1,0077,1,0,START,3,0,0"), "Kamera 4 ohne Antwortalter");
    if (getenv("SD_SIM_KEEP")) printf("    Karte: %s\n", root.c_str());
}

static void scenarioSlow(uint32_t delay_us, const char* title) {
    printf("\n== %s (%u ms je Operation) ==\n", title, delay_us / 1000);
    newCard("slow");
    g_us = 0;
    static SdLogger sd;
    sd = SdLogger{};
    sd.begin(1, false);
    g_card.op_delay_us = delay_us;
    Counts c;
    uint32_t ops_at_fail = 0;
    for (uint32_t ms = 1; ms <= 20000; ms++) {
        tick(sd, ms, c, true);
        if (sd.status() == SdLogStatus::FAILED && !ops_at_fail) ops_at_fail = g_card.ops;
    }
    check(sd.status() == SdLogStatus::FAILED && sd.failReason() == SdLogFail::SLOW, "Karte aus, Grund SLOW");
    check(ops_at_fail && g_card.ops == ops_at_fail, "danach kein einziger Kartenzugriff mehr");
}

static void scenarioWriteError() {
    printf("\n== Schreibfehler ==\n");
    newCard("werr");
    g_us = 0;
    static SdLogger sd;
    sd = SdLogger{};
    sd.begin(1, false);
    Counts c;
    for (uint32_t ms = 1; ms <= 3000; ms++) tick(sd, ms, c, true);
    g_card.fail_write = true;
    for (uint32_t ms = 3001; ms <= 6000; ms++) tick(sd, ms, c, true);
    const uint32_t ops = g_card.ops;
    for (uint32_t ms = 6001; ms <= 9000; ms++) tick(sd, ms, c, true);
    check(sd.status() == SdLogStatus::FAILED && sd.failReason() == SdLogFail::WRITE, "Karte aus, Grund WRITE");
    check(g_card.ops == ops, "danach kein Kartenzugriff mehr");
}

static void scenarioNoCard() {
    printf("\n== Keine Karte ==\n");
    newCard("nocard");
    g_card.present = false;
    g_us = 0;
    static SdLogger sd;
    sd = SdLogger{};
    check(!sd.begin(1, false), "begin() meldet keine Karte");
    Counts c;
    for (uint32_t ms = 1; ms <= 3000; ms++) tick(sd, ms, c, true);
    check(sd.status() == SdLogStatus::NO_CARD, "Status NO_CARD");
    check(sd.fillPercent() == 0, "Puffer bleibt leer");
    check(g_card.ops == SdLogger::SD_BEGIN_TRIES, "nur die Versuche in begin() (" + std::to_string(g_card.ops) + ")");
}

static void scenarioDisabled() {
    printf("\n== Boot-Guard: Karte aus ==\n");
    newCard("disabled");
    g_us = 0;
    static SdLogger sd;
    sd = SdLogger{};
    check(!sd.begin(1, true), "begin(disabled) bindet nichts ein");
    Counts c;
    for (uint32_t ms = 1; ms <= 2000; ms++) tick(sd, ms, c, true);
    check(sd.status() == SdLogStatus::DISABLED && g_card.ops == 0, "kein einziger Kartenzugriff");
}

static void scenarioOverflow() {
    printf("\n== Puffer laeuft voll (60 s keine Kartenzugriffe) ==\n");
    const std::string root = newCard("overflow");
    g_us = 0;
    static SdLogger sd;
    sd = SdLogger{};
    sd.begin(7, false);
    Counts c;
    for (uint32_t ms = 1; ms <= 60000; ms++) tick(sd, ms, c, false);
    check(sd.dropped() > 0, "verworfen gezaehlt: " + std::to_string(sd.dropped()));
    check(sd.fillPercent() >= 99, "Puffer voll");
    for (uint32_t ms = 60001; ms <= 80000; ms++) { g_us = ms * 1000ULL; sd.service(ms, true); }
    check(sd.status() == SdLogStatus::ACTIVE, "danach normal aktiv");
    const std::string s = findSession(root);
    const long rows = csvRows(s + "/IMU.csv", "IMU.csv nach Ueberlauf");
    check(rows > 0 && rows < c.imu, "IMU: Teil der Daten da (" + std::to_string(rows) + " von " + std::to_string(c.imu) + ")");
    csvRows(s + "/ARM_RAW.csv", "ARM_RAW.csv nach Ueberlauf");
}

static void scenarioSameSecond() {
    printf("\n== Zwei Starts mit gleichem Namen ==\n");
    const std::string root = newCard("same");
    for (int run = 0; run < 2; run++) {
        g_us = 0;
        static SdLogger sd;
        sd = SdLogger{};
        sd.begin(5, false);
        Counts c;
        for (uint32_t ms = 1; ms <= 3000; ms++) tick(sd, ms, c, true);
        check(sd.status() == SdLogStatus::ACTIVE, std::string("Start ") + (run ? "2" : "1") + ": " +
              (sd.sessionPath() + 8));
    }
    struct stat st;
    check(stat((root + "/MAGGIE/2026-11-19/094512_B00005_1").c_str(), &st) == 0, "zweiter Ordner mit Suffix _1");
}

int main() {
    Log.verbose = getenv("SD_SIM_VERBOSE") != nullptr;
    scenarioNormal();
    scenarioSlow(200000, "Langsame Karte");
    scenarioSlow(700000, "Sehr langsame Karte");
    scenarioWriteError();
    scenarioNoCard();
    scenarioDisabled();
    scenarioOverflow();
    scenarioSameSecond();
    printf("\n%s - %d Pruefung(en) fehlgeschlagen.\n", g_failed ? "FEHLER" : "ALLES OK", g_failed);
    return g_failed ? 1 : 0;
}
