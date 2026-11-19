// Host-Simulation der Flug-Trajektorie (ArmFlightPlayer + ArmLink, echter
// Firmware-Code) gegen eine nachgebaute RCU, die das LISA-UART-Protokoll
// spricht. Bauen und starten: tools/flight_sim/run.sh
//
// Geprueft wird vor allem, was am Boden mit allen Gelenken nicht geht:
//   - Die gesendeten Ziele liegen IMMER auf der geplanten Bahn (jede Zielrunde
//     ist genau eine Tabellenzeile, alle Gelenke zusammen, nie nur ein Teil).
//   - Die Gelenke weichen raeumlich kaum von der Bahn ab (Pfadabweichung =
//     Abstand der Ist-Stellung zur naechsten Tabellenzeile).
//   - Fehlerfaelle: stummes Gelenk, Gelenk-Reset, toter Knoten, OBC-Neustart
//     mit und ohne gesicherten Schritt, SD-Haenger, defekte Tabellen.

#include "Arduino.h"
#include "hal/arm_hal.hpp"
#include "arm_flight_player.hpp"
#include "microgravity_detector.hpp"

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Zeit und Konsole
// ---------------------------------------------------------------------------

static uint32_t g_now = 0;
// Waehrend poll() laeuft millis() eine ms vor - genau die Lage, die den
// NO_STATE-Fehler vom 17.11.2026 ausgeloest hat (state_ms > now).
static bool g_in_poll = false;
uint32_t millis() { return g_now + (g_in_poll ? 1 : 0); }

FakeConsole Serial;
void FakeConsole::printf(const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (strncmp(buf, "WARN", 4) == 0) warnings++;
    if (verbose) fputs(buf, stdout);
}
void FakeConsole::println(const char* s) {
    if (strncmp(s, "WARN", 4) == 0) warnings++;
    if (verbose) { fputs(s, stdout); fputc('\n', stdout); }
}

static constexpr double URAD_PER_DEG = 17453.2925;

// ---------------------------------------------------------------------------
// RCU-Nachbau
// ---------------------------------------------------------------------------

struct SimJoint {
    double   pos_urad   = 0;
    double   target     = 0;
    double   vel_urad_s = 6 * URAD_PER_DEG;   // RCU-Standard 6 Grad/s
    bool     released   = false;
    bool     dead       = false;              // meldet nie, faehrt nie
    uint32_t silent_from = 0, silent_to = 0;  // meldet in [from, to) nicht
};

class SimRcu : public HardwareSerial {
public:
    std::map<uint8_t, SimJoint> joints;
    int    tx_capacity = 512 + 64;   // serial1_tx_extra + Hardware-FIFO
    double tx_pending  = 0;          // Bytes noch nicht ueber die Leitung
    int    tx_overflow = 0;          // Frames, die nicht mehr passten (darf nie passieren)
    uint32_t state_period_ms = 50;

    // Zielrunde des laufenden Loops: Node -> MOVE_ABS-Ziel
    std::map<uint8_t, int32_t> burst;
    int crc_errors = 0;

    void   begin(uint32_t) override {}
    int    available() override { return static_cast<int>(rx_.size()); }
    int    read() override {
        if (rx_.empty()) return -1;
        const uint8_t b = rx_.front();
        rx_.pop_front();
        return b;
    }
    int    availableForWrite() override { return tx_capacity - static_cast<int>(tx_pending); }
    size_t write(const uint8_t* buf, size_t len) override {
        if (static_cast<int>(len) > availableForWrite()) tx_overflow++;
        tx_pending += len;
        in_.insert(in_.end(), buf, buf + len);
        parse();
        return len;
    }

    void tick(uint32_t dt_ms) {
        tx_pending = std::max(0.0, tx_pending - 11.52 * dt_ms);   // 115200 Baud
        for (auto& [node, j] : joints) {
            if (j.dead || !j.released) continue;
            const double step = j.vel_urad_s * dt_ms / 1000.0;
            const double d = j.target - j.pos_urad;
            j.pos_urad += std::clamp(d, -step, step);
        }
        if (g_now - last_state_ms_ >= state_period_ms) {
            last_state_ms_ = g_now;
            for (auto& [node, j] : joints) {
                if (j.dead) continue;
                if (g_now >= j.silent_from && g_now < j.silent_to) continue;
                sendJointState(node, j);
            }
        }
    }

    /// Gelenk startet neu: gesperrt, Ziel = Ist.
    void resetJoint(uint8_t node) {
        auto& j = joints[node];
        j.released = false;
        j.target = j.pos_urad;
    }
    void clearRx() { rx_.clear(); }

private:
    std::deque<uint8_t> rx_;
    std::vector<uint8_t> in_;
    uint32_t last_state_ms_ = 0;

    static int payloadLen(uint8_t id) {
        switch (id) {
            case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x09: case 0x0B:
            case 0x0C: case 0x0E: return 0;
            case 0x06: case 0x07: case 0x08: return 4;
            case 0x0A: return 8;
            default: return -1;
        }
    }
    static int32_t rd32(const uint8_t* p) {
        return static_cast<int32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24));
    }

    void parse() {
        while (!in_.empty()) {
            if (in_[0] != 0xAA) { in_.erase(in_.begin()); continue; }
            if (in_.size() < 4) return;
            const int len = payloadLen(in_[2]);
            if (len < 0) { in_.erase(in_.begin()); crc_errors++; continue; }
            const size_t flen = 4 + len + 4;
            if (in_.size() < flen) return;
            const uint32_t crc = ArmLink::crc32(in_.data(), 4 + len);
            if (static_cast<uint32_t>(rd32(&in_[4 + len])) != crc) {
                crc_errors++;
                in_.erase(in_.begin());
                continue;
            }
            handle(in_[1], in_[2], &in_[4]);
            in_.erase(in_.begin(), in_.begin() + flen);
        }
    }

    void handle(uint8_t node, uint8_t id, const uint8_t* p) {
        auto it = joints.find(node);
        if (it == joints.end()) return;
        if (id == 0x07) burst[node] = rd32(p);   // was der OBC schickt, auch an tote Gelenke
        SimJoint& j = it->second;
        if (j.dead) return;
        switch (id) {
            case 0x01: j.target = j.pos_urad; break;                  // STOP
            case 0x02: j.released = false; break;                     // INHIBIT
            case 0x03: j.released = true; break;                      // RELEASE
            case 0x07:                                                // MOVE_ABS
                if (j.released) j.target = rd32(p);
                break;
            case 0x0A: j.vel_urad_s = static_cast<uint32_t>(rd32(p)); break;   // SET_VELOCITY
            default: break;
        }
    }

    void sendJointState(uint8_t node, const SimJoint& j) {
        uint8_t f[13];
        f[0] = 0xAA; f[1] = node; f[2] = 0x14; f[3] = 0;
        f[4] = j.released ? (std::fabs(j.target - j.pos_urad) > 1 ? 11 : 10) : 9;
        const int32_t a = static_cast<int32_t>(std::lround(j.pos_urad));
        for (int i = 0; i < 4; i++) f[5 + i] = static_cast<uint8_t>(a >> (8 * i));
        const uint32_t crc = ArmLink::crc32(f, 9);
        for (int i = 0; i < 4; i++) f[9 + i] = static_cast<uint8_t>(crc >> (8 * i));
        rx_.insert(rx_.end(), f, f + 13);
    }
};

// ---------------------------------------------------------------------------
// Test-Tabellen
// ---------------------------------------------------------------------------

struct OwnedTable {
    std::vector<uint8_t> nodes;
    std::vector<int32_t> urad;   // [len][joints]
    ArmFlightTable t{};
    void finalize(bool valid = true) {
        t.valid  = valid;
        t.dt_ms  = 100;
        t.joints = static_cast<uint8_t>(nodes.size());
        t.len    = static_cast<uint16_t>(urad.size() / nodes.size());
        t.nodes  = nodes.data();
        t.urad   = urad.data();
        t.crc32  = ArmLink::crc32(reinterpret_cast<const uint8_t*>(urad.data()), urad.size() * 4);
    }
    int32_t at(int row, int k) const { return urad[row * nodes.size() + k]; }
};

/// 6 Gelenke, 40 s. Je Gelenk stow + A * (1 - cos(2 pi m t / T)) / 2 * w(t):
/// startet und endet in der Verstaupose, in Ruhe. Max. ~8 Grad/s.
/// symmetric = false: w(t) = 1 + 0,25 t, jede Stellung kommt nur einmal vor.
/// symmetric = true:  w(t) = 1, Rueckweg = Hinweg rueckwaerts (Zeile i und
/// 400 - i sind gleich) - wie eine Bahn "hin und auf demselben Weg zurueck".
static OwnedTable makeTable(bool symmetric = false) {
    OwnedTable o;
    // 7 Gelenke wie am echten Arm.
    constexpr int J = 7;
    const double stow[J] = { 0, 90, -45, 10, 0, 30, -20 };
    const double amp[J]  = { 30, 40, -25, 20, 15, -35, 25 };
    const int    m[J]    = { 1, 2, 1, 3, 2, 1, 2 };
    const int n = 401;
    o.nodes = { 1, 2, 3, 4, 5, 6, 7 };
    for (int i = 0; i < n; i++) {
        const double t = static_cast<double>(i) / (n - 1);
        for (int k = 0; k < J; k++) {
            const double w = symmetric ? 1.0 : 1.0 + 0.25 * t;
            const double deg = stow[k] + amp[k] * 0.5 * (1 - std::cos(2 * M_PI * m[k] * t)) * w;
            o.urad.push_back(static_cast<int32_t>(std::lround(deg * URAD_PER_DEG)));
        }
    }
    o.finalize();
    return o;
}

// ---------------------------------------------------------------------------
// Ein Simulationslauf
// ---------------------------------------------------------------------------

struct Result {
    bool     done = false;
    uint32_t done_ms = 0;
    float    max_err_deg = 0;
    uint8_t  fault_sticky = 0;
    double   max_path_dev_deg = 0;     // Ist-Stellung -> naechste Tabellenzeile
    double   final_dev_deg = 0;        // Ist-Stellung am Ende -> Verstaupose
    int      off_path_bursts = 0;      // Zielrunden, die keine Tabellenzeile sind
    int      partial_bursts = 0;       // Zielrunden ohne alle Gelenke
    int      max_back_rows = 0;        // groesster Ruecksprung zwischen Zielrunden
    int      tx_overflow = 0;
    bool     resumed = false;
    uint32_t pause_used_ms = 0;        // verbrauchtes Warte-Budget
};

struct Scenario {
    std::string name;
    std::function<void(SimRcu&)> setup = [](SimRcu&) {};
    std::function<void(SimRcu&, uint32_t t_rel)> during = [](SimRcu&, uint32_t) {};
    int32_t  reboot_at_ms = -1;        // OBC-Neustart (relativ zum Start)
    uint32_t reboot_ms = 3000;         // Dauer bis die Firmware wieder laeuft
    bool     reboot_saved_valid = true;
    uint32_t stall_every_ms = 0;       // SD-Haenger alle ... ms
    uint32_t stall_ms = 250;
    double   start_offset_deg = 0;     // Gelenke stehen nicht genau in der Startpose
};

static int findRow(const OwnedTable& o, const std::map<uint8_t, int32_t>& burst, int hint) {
    const int len = o.t.len;
    auto matches = [&](int r) {
        for (size_t k = 0; k < o.nodes.size(); k++) {
            auto it = burst.find(o.nodes[k]);
            if (it == burst.end() || it->second != o.at(r, k)) return false;
        }
        return true;
    };
    for (int d = 0; d < len; d++) {   // nahe beim letzten Treffer zuerst
        if (hint + d < len && matches(hint + d)) return hint + d;
        if (hint - d >= 0 && matches(hint - d)) return hint - d;
    }
    return -1;
}

static double pathDeviation(const OwnedTable& o, const SimRcu& rcu, int around) {
    double best = 1e9;
    const int lo = std::max(0, around - 40), hi = std::min<int>(o.t.len - 1, around + 40);
    for (int r = lo; r <= hi; r++) {
        double worst = 0;
        for (size_t k = 0; k < o.nodes.size(); k++) {
            const auto& j = rcu.joints.at(o.nodes[k]);
            if (j.dead) continue;
            worst = std::max(worst, std::fabs(j.pos_urad - o.at(r, k)) / URAD_PER_DEG);
        }
        best = std::min(best, worst);
    }
    return best;
}

static Result run(const OwnedTable& o, const Scenario& sc) {
    Result res;
    g_now = 1000;
    SimRcu rcu;
    for (size_t k = 0; k < o.nodes.size(); k++) {
        SimJoint j;
        j.pos_urad = o.at(0, k) + sc.start_offset_deg * URAD_PER_DEG * (k % 2 ? 1 : -1);
        j.target = j.pos_urad;
        rcu.joints[o.nodes[k]] = j;
    }
    sc.setup(rcu);

    auto link = std::make_unique<ArmLink>(rcu);
    auto player = std::make_unique<ArmFlightPlayer>(o.t);
    link->init(115200);
    if (!player->verifyTable()) { fprintf(stderr, "Tabelle abgelehnt\n"); return res; }

    // Ein paar Loops ohne Start, damit Gelenkmeldungen da sind.
    const uint32_t t0 = g_now;
    bool started = false;
    uint16_t saved_step = 0;
    uint32_t last_ckpt = 0;
    int last_row = 0, hint = 0;
    uint32_t next_stall = sc.stall_every_ms ? t0 + sc.stall_every_ms : 0;
    bool rebooted = false;

    const uint32_t limit = t0 + 120000;
    while (g_now < limit) {
        const uint32_t rel = g_now - t0;
        sc.during(rcu, rel);

        // OBC-Neustart: Firmware weg, RCU und Gelenke laufen weiter.
        if (!rebooted && sc.reboot_at_ms >= 0 && rel >= static_cast<uint32_t>(sc.reboot_at_ms)) {
            rebooted = true;
            player.reset();
            link.reset();
            for (uint32_t i = 0; i < sc.reboot_ms; i++) { g_now++; rcu.tick(1); }
            rcu.clearRx();   // was waehrend des Neustarts kam, ist weg
            link = std::make_unique<ArmLink>(rcu);
            link->init(115200);
            player = std::make_unique<ArmFlightPlayer>(o.t);
            player->verifyTable();
            for (int i = 0; i < 5; i++) { g_in_poll = true; link->poll(); g_in_poll = false; g_now++; rcu.tick(1); }
            player->resume(saved_step, sc.reboot_saved_valid, g_now);
            continue;
        }

        // SD-Haenger: der Loop steht, alles andere laeuft.
        if (next_stall && g_now >= next_stall) {
            for (uint32_t i = 0; i < sc.stall_ms; i++) { g_now++; rcu.tick(1); }
            next_stall += sc.stall_every_ms;
        }

        rcu.burst.clear();
        g_in_poll = true;
        link->poll();
        g_in_poll = false;
        if (!started && rel >= 200) started = player->start(*link, g_now);
        player->update(*link, g_now);

        if (!rcu.burst.empty() && player->phase() != ArmFlightPlayer::Phase::APPROACH) {
            if (rcu.burst.size() != o.nodes.size()) res.partial_bursts++;
            const int row = findRow(o, rcu.burst, hint);
            if (row < 0) {
                res.off_path_bursts++;
            } else {
                res.max_back_rows = std::max(res.max_back_rows, last_row - row);
                last_row = row;
                hint = row;
            }
        }

        // Checkpoint wie System::updateCheckpoint (jede volle Sekunde).
        if (player->active() && g_now - last_ckpt >= 1000) {
            last_ckpt = g_now;
            saved_step = player->step();
        }

        if (player->phase() == ArmFlightPlayer::Phase::RUN && g_now % 100 == 0) {
            res.max_path_dev_deg = std::max(res.max_path_dev_deg, pathDeviation(o, rcu, player->step()));
        }
        if (player->done()) {
            res.done = true;
            res.done_ms = g_now - t0;
            break;
        }
        g_now++;
        rcu.tick(1);
    }

    for (size_t k = 0; k < o.nodes.size(); k++) {
        const auto& j = rcu.joints.at(o.nodes[k]);
        if (j.dead) continue;
        res.final_dev_deg = std::max(res.final_dev_deg,
                                     std::fabs(j.pos_urad - o.at(o.t.len - 1, k)) / URAD_PER_DEG);
    }
    res.max_err_deg  = player ? player->maxErrDeg() : 0;
    res.pause_used_ms = player ? player->pauseUsedMs() : 0;
    res.fault_sticky = player ? player->faultSticky() : 0;
    res.resumed      = player && player->resumed();
    res.tx_overflow  = rcu.tx_overflow;
    return res;
}

// ---------------------------------------------------------------------------
// Pruefungen
// ---------------------------------------------------------------------------

static int g_failed = 0;
static void check(bool ok, const std::string& what) {
    printf("    [%s] %s\n", ok ? " OK " : "FAIL", what.c_str());
    if (!ok) g_failed++;
}

static void report(const char* name, const Result& r) {
    printf("\n%s\n", name);
    printf("    fertig=%s nach %.1f s, max. Schleppfehler %.2f Grad, Pfadabweichung %.2f Grad, "
           "Endabweichung %.3f Grad, Fehlermaske 0x%02X, Ruecksprung %d Zeilen\n",
           r.done ? "ja" : "NEIN", r.done_ms / 1000.0, r.max_err_deg, r.max_path_dev_deg,
           r.final_dev_deg, r.fault_sticky, r.max_back_rows);
}

static void commonChecks(const Result& r, double max_path_dev) {
    check(r.done, "Bahn bis zum Ende abgefahren (DONE)");
    check(r.off_path_bursts == 0, "jede Zielrunde ist eine Tabellenzeile (" +
          std::to_string(r.off_path_bursts) + " Abweichungen)");
    check(r.partial_bursts == 0, "nie nur ein Teil der Gelenke angesteuert");
    check(r.tx_overflow == 0, "TX-Puffer nie uebergelaufen");
    check(r.max_path_dev_deg <= max_path_dev, "Ist-Stellung hoechstens " +
          std::to_string(max_path_dev).substr(0, 4) + " Grad neben der Bahn");
    check(r.final_dev_deg <= ArmFlightPlayer::DONE_TOL_DEG, "Ende in der Verstaupose");
}

static void testDetector() {
    printf("\nSchwerelosigkeits-Erkennung\n");
    MicrogravityDetector d;
    uint32_t t = 0;
    for (int i = 0; i < 300; i++, t += 10) d.feed(0.01f, -0.98f + (i % 3) * 0.001f, 0.02f, t);
    check(!d.active(t), "1 g am Boden: nie aktiv");

    MicrogravityDetector z;
    t = 0;
    for (int i = 0; i < 300; i++, t += 10) z.feed(0.0f, 0.0f, 0.0f, t);
    check(!z.active(t), "Sensor liefert nur Nullen: nie aktiv");

    MicrogravityDetector f;
    t = 0;
    for (int i = 0; i < 300; i++, t += 10) f.feed(0.01f, 0.02f, 0.01f, t);
    check(!f.active(t), "eingefrorener Wert: nie aktiv");

    MicrogravityDetector u;
    t = 0;
    for (int i = 0; i < 90; i++, t += 10) u.feed(0.01f + (i % 5) * 0.002f, 0.02f, -0.01f, t);
    check(!u.active(t), "Schwerelos erst 0,9 s: noch nicht aktiv");
    for (int i = 0; i < 20; i++, t += 10) u.feed(0.01f + (i % 5) * 0.002f, 0.02f, -0.01f, t);
    check(u.active(t), "Schwerelos 1,1 s: aktiv");
    check(!u.active(t + 500), "letzter Wert 0,5 s alt: nicht mehr aktiv");
    u.feed(0.0f, 0.5f, 0.0f, t);
    check(!u.active(t), "ein Wert ueber 0,1 g: sofort zurueckgesetzt");

    MicrogravityDetector g;
    t = 0;
    for (int i = 0; i < 60; i++, t += 10) g.feed(0.01f + (i % 5) * 0.002f, 0.02f, -0.01f, t);
    t += 300;   // Luecke
    for (int i = 0; i < 60; i++, t += 10) g.feed(0.01f + (i % 5) * 0.002f, 0.02f, -0.01f, t);
    check(!g.active(t), "Luecke von 0,3 s setzt die Haltezeit zurueck");
}

static void testTables() {
    printf("\nTabellenpruefung beim Booten\n");
    {
        OwnedTable o = makeTable();
        ArmFlightPlayer p(o.t);
        check(p.verifyTable(), "gueltige Tabelle angenommen");
    }
    {
        OwnedTable o = makeTable();
        o.t.valid = false;
        ArmFlightPlayer p(o.t);
        check(!p.verifyTable(), "Platzhalter (VALID = false) abgelehnt");
    }
    {
        OwnedTable o = makeTable();
        o.urad[100 * o.t.joints + 2] += 1;   // ein urad veraendert, CRC alt
        ArmFlightPlayer p(o.t);
        check(!p.verifyTable(), "veraenderte Tabelle (CRC passt nicht) abgelehnt");
    }
    {
        OwnedTable o = makeTable();
        o.urad[(o.t.len - 1) * o.t.joints + 1] += static_cast<int32_t>(1.0 * URAD_PER_DEG);
        o.finalize();
        ArmFlightPlayer p(o.t);
        check(!p.verifyTable(), "Ende != Startpose abgelehnt");
    }
    {
        OwnedTable o = makeTable();
        o.urad[200 * o.t.joints + 0] += static_cast<int32_t>(2.0 * URAD_PER_DEG);   // Sprung 2 Grad in 0,1 s
        o.finalize();
        ArmFlightPlayer p(o.t);
        check(!p.verifyTable(), "zu schneller Schritt (> 10 Grad/s) abgelehnt");
    }
    {
        OwnedTable o = makeTable();
        o.nodes[3] = o.nodes[1];
        o.finalize();
        ArmFlightPlayer p(o.t);
        check(!p.verifyTable(), "doppelte Node abgelehnt");
    }
    {
        OwnedTable o;
        o.nodes = { 1 };
        for (int i = 0; i < 1000; i++) o.urad.push_back(0);   // 99,9 s
        o.finalize();
        ArmFlightPlayer p(o.t);
        check(!p.verifyTable(), "laenger als 90 s abgelehnt");
    }
    {
        OwnedTable o = makeTable();
        ArmFlightPlayer p(o.t);   // verifyTable nie aufgerufen
        SimRcu rcu;
        ArmLink link(rcu);
        link.init(115200);
        check(!p.start(link, 0) && !p.resume(0, true, 0), "ohne gepruefte Tabelle kein Start");
    }
}

int main() {
    Serial.verbose = getenv("FLIGHT_SIM_VERBOSE") && getenv("FLIGHT_SIM_VERBOSE")[0] == '1';
    const OwnedTable o = makeTable();
    printf("Testbahn: %u Gelenke, %u Punkte (%.1f s)\n", o.t.joints, o.t.len, (o.t.len - 1) / 10.0);

    testDetector();
    testTables();

    {
        Scenario sc{"Normalfall"};
        Result r = run(o, sc);
        report("Normalfall", r);
        commonChecks(r, 1.0);
        check(r.fault_sticky == 0, "keine Gelenkprobleme gemeldet");
        check(r.max_back_rows == 0, "nie rueckwaerts");
    }
    {
        Scenario sc{"Gelenke 0,8 Grad neben der Startpose"};
        sc.start_offset_deg = 0.8;
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        commonChecks(r, 1.0);
    }
    {
        Scenario sc{"Node 3 meldet 3 s nicht (t = 10..13 s)"};
        sc.setup = [](SimRcu& rcu) { rcu.joints[3].silent_from = 11000; rcu.joints[3].silent_to = 14000; };
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        commonChecks(r, 1.0);
        check(r.fault_sticky == 0x04, "Problem von Node 3 (Bit 2) gemeldet");
    }
    {
        Scenario sc{"Node 2 startet neu (t = 15 s, danach gesperrt)"};
        sc.during = [](SimRcu& rcu, uint32_t rel) { if (rel == 15000) rcu.resetJoint(2); };
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        check(r.done, "Bahn bis zum Ende abgefahren (DONE)");
        check(r.off_path_bursts == 0 && r.partial_bursts == 0, "Ziele immer auf der Bahn, alle Gelenke");
        check(r.fault_sticky & 0x02, "Schleppfehler von Node 2 (Bit 1) gemeldet");
        check(r.final_dev_deg <= ArmFlightPlayer::DONE_TOL_DEG,
              "Node 2 per RELEASE wieder eingefangen, Ende in der Verstaupose");
    }
    {
        Scenario sc{"Node 4 tot (meldet nie, faehrt nie)"};
        sc.setup = [](SimRcu& rcu) { rcu.joints[4].dead = true; };
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        check(r.done, "Bahn mit den uebrigen Gelenken bis zum Ende abgefahren");
        check(r.off_path_bursts == 0 && r.partial_bursts == 0, "Ziele immer auf der Bahn, alle Gelenke");
        check(r.fault_sticky == 0x08, "Node 4 (Bit 3) gemeldet");
    }
    {
        // Bodentest mit voller Flug-Tabelle, aber nur zwei eingebauten Gelenken.
        Scenario sc{"Nur Node 1 und 2 eingebaut (3..7 fehlen)"};
        sc.setup = [](SimRcu& rcu) { for (int n = 3; n <= 7; ++n) rcu.joints[n].dead = true; };
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        check(r.done, "Bahn mit Node 1 und 2 bis zum Ende abgefahren");
        check(r.off_path_bursts == 0 && r.partial_bursts == 0, "Ziele immer auf der Bahn, alle Gelenke");
        check(r.fault_sticky == 0x7C, "Node 3..7 (Bits 2..6) gemeldet");
        check(r.pause_used_ms == 0, "fehlende Gelenke loesen kein Warten aus");
    }
    {
        Scenario sc{"OBC-Neustart bei t = 18,3 s (3 s weg), Schritt gesichert"};
        sc.reboot_at_ms = 18300;
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        commonChecks(r, 1.0);
        check(r.resumed, "fortgesetzt statt neu gestartet");
        check(r.max_back_rows <= 3, "nach dem Neustart hoechstens 3 Zeilen zurueck");
    }
    {
        Scenario sc{"OBC-Neustart bei t = 25 s OHNE gesicherten Schritt"};
        sc.reboot_at_ms = 25000;
        sc.reboot_saved_valid = false;
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        commonChecks(r, 1.0);
        check(r.max_back_rows <= 3, "Suche ueber die ganze Tabelle findet die richtige Stelle");
    }
    {
        // Hinweg = Rueckweg: Ohne gesicherten Schritt ist die Stelle nicht
        // eindeutig. Gewaehlt wird die fruehere - ein Stueck wird wiederholt,
        // aber die Stellung bleibt auf der Bahn (keine Kollision).
        const OwnedTable sym = makeTable(true);
        Scenario sc{"Hinweg = Rueckweg, Neustart bei t = 25 s OHNE gesicherten Schritt"};
        sc.reboot_at_ms = 25000;
        sc.reboot_saved_valid = false;
        Result r = run(sym, sc);
        report(sc.name.c_str(), r);
        commonChecks(r, 1.0);
        check(r.done_ms < 60000, "wiederholtes Stueck kostet hoechstens ~10 s");
    }
    {
        Scenario sc{"SD-Haenger: alle 2 s steht der Loop 250 ms"};
        sc.stall_every_ms = 2000;
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        commonChecks(r, 1.5);
    }
    {
        Scenario sc{"Knapper TX-Puffer (300 Byte, passt nur eine Zielrunde)"};
        sc.setup = [](SimRcu& rcu) { rcu.tx_capacity = 300; };
        Result r = run(o, sc);
        report(sc.name.c_str(), r);
        commonChecks(r, 1.5);
    }

    printf("\n%s - %d Pruefung(en) fehlgeschlagen.\n", g_failed ? "FEHLER" : "ALLES OK", g_failed);
    return g_failed ? 1 : 0;
}
