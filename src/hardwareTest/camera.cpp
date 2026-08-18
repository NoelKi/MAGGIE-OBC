#include <Arduino.h>

#include "pin_config.hpp"
#include "camera_config.hpp"   // CAMERA_BUS_UART - selber UART wie im Flugbetrieb
#include "hal/camera_bus.hpp"
#include "hal/camera_hal.hpp"

/**
 * @file hardwareTest/camera.cpp
 * @brief Interaktiver Hardware-Test für die 4 RunCam Split 4 am Mux.
 *
 * Aktivieren in platformio.ini:
 *     build_src_filter =
 *         ...
 *         ; +<main.cpp>              <- auskommentieren (doppeltes setup/loop)
 *         +<hardwareTest/camera.cpp>
 *
 * Danach seriellen Monitor öffnen (115200) und 'h' für die Hilfe senden.
 *
 * Auswahl: mit '1'..'4' eine einzelne Kamera anwählen, mit 'a' alle. Jedes
 * Kommando geht an die aktuelle Auswahl - bei 'a' nacheinander an jeden
 * Mux-Kanal, mit dem normalen Rate-Limit dazwischen.
 *
 * Der Test benutzt bewusst CameraTriggerMode::MANUAL_ONLY: es passiert nichts,
 * was nicht ausgelöst wurde. Die echte Flugkonfiguration steht in
 * camera_config.hpp und startet die Aufnahme automatisch.
 */

static constexpr uint8_t TEST_CAMERA_COUNT = 4;

// Kürzeres Boot-Delay als im Flug: beim Test hängt die Kamera meist schon
// länger am Strom als der Teensy. Antwortet sie nicht, hilft 'i'.
static constexpr uint32_t TEST_BOOT_DELAY_MS = 2000;

static CameraBus bus(CAMERA_BUS_UART, PIN_CAM_MUX_A, PIN_CAM_MUX_B);

static CameraHAL* cameras[TEST_CAMERA_COUNT] = {};

/// 0 = alle Kameras, sonst 1..4 = genau diese Kamera.
static uint8_t selection = 0;

static void printHelp() {
    Serial.println();
    Serial.println("=== RunCam Split 4 Hardware-Test ===============================");
    Serial.println("  Auswahl:");
    Serial.println("    1..4  Kamera 1..4 einzeln anwaehlen");
    Serial.println("    a     alle Kameras gleichzeitig ansprechen");
    Serial.println("  Kommandos an die Auswahl:");
    Serial.println("    r     Aufnahme starten   (CC 01 01 E7, Power-Btn-Toggle)");
    Serial.println("    s     Aufnahme stoppen   (CC 01 01 E7, Power-Btn-Toggle)");
    Serial.println("    t     rohes Toggle senden, ohne Zustandsverwaltung");
    Serial.println("    m     Modus wechseln     (CC 01 02 4D, Video <-> OSD)");
    Serial.println("    w     WiFi-Taste         (CC 01 00 32)");
    Serial.println("    i     Device-Info-Handshake erneut ausfuehren");
    Serial.println("  Sonstiges:");
    Serial.println("    p     alle 4 Mux-Kanaele durchscannen");
    Serial.println("    ?     Status aller Kameras");
    Serial.println("    h     diese Hilfe");
    Serial.println("================================================================");
    Serial.println();
}

static const char* stateName(CameraState state) {
    switch (state) {
        case CameraState::UNCONFIGURED: return "UNCONFIGURED";
        case CameraState::BOOTING:      return "BOOTING";
        case CameraState::DETECTING:    return "DETECTING";
        case CameraState::READY:        return "READY";
    }
    return "?";
}

static void printStatus() {
    Serial.println();
    if (selection == 0) {
        Serial.println("Auswahl: ALLE Kameras");
    } else {
        Serial.printf("Auswahl: nur Kamera %u\n", selection);
    }
    Serial.printf("Mux-Kanal aktuell: %u\n", bus.currentChannel());
    Serial.println("ID  Kanal  Zustand       Erkannt  Aufnahme  ProtoV  Features");
    for (uint8_t i = 0; i < TEST_CAMERA_COUNT; ++i) {
        CameraHAL* cam = cameras[i];
        Serial.printf("%2u  %5u  %-12s  %-7s  %-8s  %6u  0x%04X\n",
                      cam->getCameraID(), cam->getMuxChannel(),
                      stateName(cam->getState()),
                      cam->isDetected() ? "ja" : "NEIN",
                      cam->isRecording() ? "laeuft" : "aus",
                      cam->getProtocolVersion(), cam->getFeatures());
    }
    Serial.println();
}

/// Führt action für jede angewählte Kamera aus und meldet das Ergebnis.
static void forEachSelected(const char* label, bool (*action)(CameraHAL*)) {
    for (uint8_t i = 0; i < TEST_CAMERA_COUNT; ++i) {
        CameraHAL* cam = cameras[i];
        if (selection != 0 && cam->getCameraID() != selection) continue;

        const bool ok = action(cam);
        Serial.printf("  Kamera %u (Kanal %u): %s -> %s\n",
                      cam->getCameraID(), cam->getMuxChannel(), label,
                      ok ? "gesendet" : "FEHLGESCHLAGEN");

        // Das Rate-Limit im HAL greift pro Kamera. Bei 'a' laufen die
        // Kommandos ohnehin ueber verschiedene Mux-Kanaele, ein kurzer
        // Abstand macht den Ablauf im Monitor aber nachvollziehbar.
        delay(50);
    }
}

/// Schaltet alle 4 Kanäle durch und probiert je einen Handshake.
static void scanChannels() {
    Serial.println();
    Serial.println("Scanne Mux-Kanaele 0..3 ...");
    for (uint8_t i = 0; i < TEST_CAMERA_COUNT; ++i) {
        CameraHAL* cam = cameras[i];
        const bool ok = cam->probe();
        Serial.printf("  Kanal %u (Kamera %u): %s",
                      cam->getMuxChannel(), cam->getCameraID(),
                      ok ? "Antwort OK" : "keine gueltige Antwort");
        if (ok) {
            Serial.printf("  -> Protokoll v%u, Features 0x%04X",
                          cam->getProtocolVersion(), cam->getFeatures());
        }
        Serial.println();
    }
    Serial.println("Scan fertig.");
    Serial.println();
}

static void handleKey(char key) {
    switch (key) {
        case '1': case '2': case '3': case '4':
            selection = static_cast<uint8_t>(key - '0');
            Serial.printf("\n>> Auswahl: nur Kamera %u\n\n", selection);
            break;

        case 'a': case 'A':
            selection = 0;
            Serial.println("\n>> Auswahl: ALLE Kameras\n");
            break;

        case 'r': case 'R':
            Serial.println("\n>> Aufnahme starten");
            forEachSelected("startRecording", [](CameraHAL* c) { return c->startRecording(); });
            Serial.println();
            break;

        case 's': case 'S':
            Serial.println("\n>> Aufnahme stoppen");
            forEachSelected("stopRecording", [](CameraHAL* c) { return c->stopRecording(); });
            Serial.println();
            break;

        case 't': case 'T':
            Serial.println("\n>> Rohes Power-Btn-Toggle (Zustand im HAL bleibt unveraendert)");
            forEachSelected("SIMULATE_POWER_BTN", [](CameraHAL* c) {
                return c->sendControlAction(RunCam::Action::SIMULATE_POWER_BTN);
            });
            Serial.println();
            break;

        case 'm': case 'M':
            Serial.println("\n>> Modus wechseln (Video <-> OSD)");
            forEachSelected("changeMode", [](CameraHAL* c) { return c->changeMode(); });
            Serial.println();
            break;

        case 'w': case 'W':
            Serial.println("\n>> WiFi-Taste");
            forEachSelected("SIMULATE_WIFI_BTN", [](CameraHAL* c) {
                return c->sendControlAction(RunCam::Action::SIMULATE_WIFI_BTN);
            });
            Serial.println();
            break;

        case 'i': case 'I':
            Serial.println("\n>> Device-Info-Handshake");
            forEachSelected("GET_DEVICE_INFO", [](CameraHAL* c) { return c->probe(); });
            Serial.println();
            break;

        case 'p': case 'P':
            scanChannels();
            break;

        case '?':
            printStatus();
            break;

        case 'h': case 'H':
            printHelp();
            break;

        default:
            break;  // Zeilenumbrueche und Tippfehler ignorieren
    }
}

void setup() {
    Serial.begin(115200);
    const uint32_t start = millis();
    while (!Serial && (millis() - start) < 3000) {
        // Auf den USB-Monitor warten, aber nicht ewig
    }

    Serial.println();
    Serial.println("RunCam Split 4 Hardware-Test startet...");
    Serial.printf("UART: Serial2 (Pin %u = TX2 -> Mux, Pin %u = RX2 <- Mux), %lu Baud\n",
                  PIN_CAM_MAIN_RX, PIN_CAM1_TX, (unsigned long)RunCam::BAUDRATE);
    Serial.printf("Mux-Select: Pin %u = CAMDIR1 (LSB), Pin %u = CAMDIR2 (MSB)\n",
                  PIN_CAM_MUX_A, PIN_CAM_MUX_B);

    if (!bus.begin(RunCam::BAUDRATE)) {
        Serial.println("FEHLER: Kamera-Bus konnte nicht geoeffnet werden.");
        return;
    }

    for (uint8_t i = 0; i < TEST_CAMERA_COUNT; ++i) {
        const CameraConfig config = {
            /*camera_id=*/static_cast<uint8_t>(i + 1),
            /*mux_channel=*/i,
            /*enabled=*/true,
            RunCamModel::SPLIT_4,
            CameraTriggerMode::MANUAL_ONLY,
            TEST_BOOT_DELAY_MS,
        };
        cameras[i] = new CameraHAL(config, &bus);
        cameras[i]->init();
    }

    Serial.printf("Warte %lu ms auf den Boot der Kameras, dann Handshake...\n",
                  (unsigned long)TEST_BOOT_DELAY_MS);
    printHelp();
}

void loop() {
    const uint32_t now = millis();

    // Treibt Boot-Delay und Handshake weiter. MANUAL_ONLY heisst: kein
    // automatisches Starten der Aufnahme.
    for (uint8_t i = 0; i < TEST_CAMERA_COUNT; ++i) {
        if (cameras[i]) cameras[i]->update(now);
    }

    // Ergebnis des Handshakes einmalig melden.
    static bool reported[TEST_CAMERA_COUNT] = {};
    for (uint8_t i = 0; i < TEST_CAMERA_COUNT; ++i) {
        if (!cameras[i] || reported[i]) continue;
        if (cameras[i]->getState() != CameraState::READY) continue;
        reported[i] = true;
        Serial.printf("Kamera %u (Kanal %u): %s\n",
                      cameras[i]->getCameraID(), cameras[i]->getMuxChannel(),
                      cameras[i]->isDetected() ? "erkannt" : "keine Antwort (blind)");
    }

    while (Serial.available() > 0) {
        handleKey(static_cast<char>(Serial.read()));
    }
}
