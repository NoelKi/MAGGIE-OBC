// Host-Test fuer CameraHAL: 4 nachgebaute RunCam-Kameras hinter einem
// CD4052B-Mux. Prueft Kanalwahl, CRC, Zuordnung der Antworten, Start/Stopp.
//   tools/camera_sim/run.sh
#include "hal/camera_hal.hpp"
#include "pin_config.hpp"
#include <deque>
#include <vector>
#include <cstdlib>
#include <cstring>

FakeConsole Serial;
static uint32_t g_now = 0;
uint32_t millis() { return g_now; }
static int g_pin[64] = {};
static int g_switches = 0;
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t val) {
    if (g_pin[pin] != val) ++g_switches;
    g_pin[pin] = val;
}
static uint8_t channel() { return (g_pin[PIN_CAM_SEL_A] ? 1 : 0) | (g_pin[PIN_CAM_SEL_B] ? 2 : 0); }

static uint8_t crc(const uint8_t* d, size_t n) { return CameraHAL::crc8(d, static_cast<uint8_t>(n)); }

struct SimCam {
    bool     powered = true;
    uint16_t features = 0;
    uint32_t delay_ms = 20;      // Antwortzeit
    bool     bad_crc = false;
    bool     recording = true;   // Auto-Record ab Einschalten
    int      starts = 0, stops = 0, infos = 0, garbage = 0;
};

struct Pending { uint32_t at; uint8_t cam; std::vector<uint8_t> bytes; };

class MuxSerial : public HardwareSerial {
public:
    SimCam cams[4];
    std::deque<uint8_t> rx;
    std::vector<Pending> pending;
    int tx_room = 64;
    int wrong_channel_writes = 0;
    void begin(uint32_t) override {}
    int available() override { return static_cast<int>(rx.size()); }
    int read() override { if (rx.empty()) return -1; int b = rx.front(); rx.pop_front(); return b; }
    int availableForWrite() override { return tx_room; }
    size_t write(const uint8_t* b, size_t n) override {
        SimCam& c = cams[channel()];
        if (!c.powered) return n;
        if (n >= 3 && b[0] == 0xCC && crc(b, n - 1) == b[n - 1]) {
            if (b[1] == 0x00 && n == 3) {
                ++c.infos;
                std::vector<uint8_t> r = {0xCC, 0x01, static_cast<uint8_t>(c.features & 0xFF),
                                          static_cast<uint8_t>(c.features >> 8)};
                r.push_back(static_cast<uint8_t>(crc(r.data(), r.size()) ^ (c.bad_crc ? 0x5A : 0)));
                pending.push_back({g_now + c.delay_ms, channel(), r});
            } else if (b[1] == 0x01 && n == 4) {
                if (b[2] == 0x03) { ++c.starts; c.recording = true; }
                if (b[2] == 0x04) { ++c.stops; c.recording = false; }
            } else ++c.garbage;
        } else ++c.garbage;
        return n;
    }
    void tick(bool switched) {
        if (switched) rx.push_back(0xCC);   // Stoerbyte beim Umschalten
        for (size_t i = 0; i < pending.size();) {
            if (static_cast<int32_t>(g_now - pending[i].at) >= 0) {
                // Antwort kommt nur an, wenn der Mux noch auf dieser Kamera steht.
                if (pending[i].cam == channel()) for (uint8_t x : pending[i].bytes) rx.push_back(x);
                pending.erase(pending.begin() + i);
            } else ++i;
        }
    }
};

static int g_fail = 0;
static void check(bool ok, const char* what) {
    printf("    [%s] %s\n", ok ? " OK " : "FEHL", what);
    if (!ok) ++g_fail;
}

static void run(CameraHAL& cam, MuxSerial& s, uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
        const int sw = g_switches;
        cam.update(g_now);
        s.tick(g_switches != sw);
        ++g_now;
    }
}

int main() {
    Serial.verbose = getenv("CAM_SIM_VERBOSE") && getenv("CAM_SIM_VERBOSE")[0] == '1';

    printf("CRC-8/DVB-S2\n");
    const char* chk = "123456789";
    check(CameraHAL::crc8(reinterpret_cast<const uint8_t*>(chk), 9) == 0xBC, "Pruefwert \"123456789\" = 0xBC");

    printf("\nKamera 1, 2, 4 an (verschiedene Features), Kamera 3 aus\n");
    g_now = 1000;
    MuxSerial s;
    s.cams[0].features = 0x00C1;
    s.cams[1].features = 0x00C3;
    s.cams[2].powered  = false;
    s.cams[3].features = 0x0047;
    s.cams[3].delay_ms = 250;     // langsam, aber im Zeitfenster
    CameraHAL cam(s, PIN_CAM_SEL_A, PIN_CAM_SEL_B);
    cam.init(115200);
    run(cam, s, 6000);
    check(cam.alive(0, g_now) && cam.alive(1, g_now) && cam.alive(3, g_now), "Kamera 1, 2, 4 antworten");
    check(!cam.alive(2, g_now) && cam.status(2).probes_failed > 0, "Kamera 3 antwortet nicht, Fehlversuche gezaehlt");
    check(cam.status(0).features == 0x00C1 && cam.status(1).features == 0x00C3 &&
          cam.status(3).features == 0x0047, "Features der richtigen Kamera zugeordnet");
    check(s.cams[0].garbage == 0 && s.cams[1].garbage == 0 && s.cams[3].garbage == 0,
          "keine kaputten Pakete bei den Kameras");

    printf("\nSicherheits-Start an alle, dann Stopp nur Kamera 2\n");
    for (auto& c : s.cams) c.starts = c.stops = 0;
    cam.requestAll(CamAction::START);
    run(cam, s, 200);
    check(s.cams[0].starts == 1 && s.cams[1].starts == 1 && s.cams[3].starts == 1,
          "jede laufende Kamera bekommt genau ein START");
    check(s.cams[0].recording && s.cams[1].recording && s.cams[3].recording, "Aufnahme laeuft weiter");
    check(cam.status(0).last_cmd == CamAction::START, "zuletzt gesendet: START");
    cam.request(1, CamAction::STOP);
    run(cam, s, 200);
    check(s.cams[1].stops == 1 && !s.cams[1].recording, "Kamera 2 gestoppt");
    check(s.cams[0].stops == 0 && s.cams[3].stops == 0 && s.cams[0].recording && s.cams[3].recording,
          "die anderen laufen weiter");

    printf("\nSTART und STOP kurz hintereinander: der letzte Wunsch zaehlt\n");
    for (auto& c : s.cams) c.starts = c.stops = 0;
    cam.request(0, CamAction::STOP);
    cam.request(0, CamAction::START);
    run(cam, s, 200);
    check(s.cams[0].starts == 1 && s.cams[0].stops == 0 && s.cams[0].recording, "nur START gesendet");

    printf("\nKaputte Antwort (CRC falsch) und zu langsame Kamera\n");
    MuxSerial s2;
    s2.cams[0].bad_crc = true;
    s2.cams[1].delay_ms = 400;
    s2.cams[2].powered = false;
    s2.cams[3].powered = false;
    CameraHAL cam2(s2, PIN_CAM_SEL_A, PIN_CAM_SEL_B);
    cam2.init(115200);
    run(cam2, s2, 6000);
    check(!cam2.status(0).answered_ever, "Antwort mit falscher CRC verworfen");
    check(!cam2.status(1).answered_ever, "Antwort nach 400 ms (Mux schon weiter) nicht falsch zugeordnet");
    check(cam2.status(0).rx_any && cam2.status(0).rx_garbage, "Diagnose: Kamera 1 schickt Bytes, aber ungueltig");
    check(!cam2.status(2).rx_any && !cam2.status(2).rx_garbage, "Diagnose: Kamera 3 (aus) schickt nichts");

    printf("\nVoller TX-Puffer: Befehl wartet, geht danach raus\n");
    s.tx_room = 0;
    for (auto& c : s.cams) c.starts = 0;
    cam.request(3, CamAction::START);
    run(cam, s, 50);
    check(s.cams[3].starts == 0, "nichts gesendet, solange kein Platz");
    s.tx_room = 64;
    run(cam, s, 50);
    check(s.cams[3].starts == 1, "danach genau einmal gesendet");

    printf("\nmillis()-Ueberlauf\n");
    g_now = 0xFFFFFFFFu - 3000;
    MuxSerial s3;
    CameraHAL cam3(s3, PIN_CAM_SEL_A, PIN_CAM_SEL_B);
    cam3.init(115200);
    run(cam3, s3, 8000);
    check(cam3.alive(0, g_now) && cam3.alive(1, g_now) && cam3.alive(2, g_now) && cam3.alive(3, g_now),
          "alle 4 antworten auch ueber den Ueberlauf hinweg");

    printf("\n%s - %d Pruefung(en) fehlgeschlagen.\n", g_fail ? "FEHLER" : "ALLES OK", g_fail);
    return g_fail ? 1 : 0;
}
