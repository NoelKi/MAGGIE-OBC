#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>

#include "hal/camera_bus.hpp"

/**
 * @file camera_hal.hpp
 * @brief RunCam Device Protocol (UART) für RunCam Split 4
 *
 * Physical Layer
 * --------------
 *   115200 Baud, 8N1, 3.3V TTL (Teensy 4.1 ist 3.3V - direkt kompatibel,
 *   KEIN Levelshifter nötig, Teensy-Pins sind aber NICHT 5V-tolerant!).
 *   Verdrahtung: Teensy TXn -> Kamera RX, Teensy RXn <- Kamera TX, GND gemeinsam.
 *   Bei MAGGIE hängen 4 Kameras über einen 4:1-Mux an einem UART, siehe
 *   camera_bus.hpp - immer nur eine Kamera ist gleichzeitig erreichbar.
 *   Kamera-Versorgung laut Handbuch DC 5-20V, >= 1A, NICHT direkt an der
 *   Batterie und nicht über den VTx (Spannungsspitzen zerstören die Kamera).
 *
 * Paketformat (Host -> Kamera)
 * ----------------------------
 *   [0xCC] [Command] [Action ...] [CRC8]
 *   CRC8 = DVB-S2 (Polynom 0xD5, Startwert 0x00) über ALLE vorherigen Bytes
 *   inklusive des Headers 0xCC.
 *
 *   Beispiel "Start Recording": CC 01 03 98
 *
 * Kamerasteuerung (Command 0x01) quittiert NICHT - es kommt keine Antwort
 * zurück. Nur GET_DEVICE_INFO (0x00) antwortet (5 Bytes).
 *
 * WICHTIG - Eigenheit der Split 4
 * -------------------------------
 * Die Split 4 setzt die dedizierten Aktionen START_RECORDING (0x03) und
 * STOP_RECORDING (0x04) NICHT um, obwohl sie die passenden Feature-Bits
 * meldet. Aufnahme wird stattdessen über SIMULATE_POWER_BTN (0x01)
 * umgeschaltet - das ist ein TOGGLE, kein Start/Stop.
 * Siehe ArduPilot AP_RunCam (RunCamModel::Split4k -> SIMULATE_POWER_BTN)
 * und das Split-4-Handbuch ("CAMERA POWER: start/stop the video").
 * Deshalb muss der Aufnahmezustand in der Firmware mitgeführt werden.
 *
 * Ebenfalls wichtig: Das Protokoll kennt KEIN Foto-/Snapshot-Kommando. Die
 * Split 4 ist reines Video (MP4). Ein "alle N Sekunden ein Snapshot"-Modus
 * ist mit dieser Kamera nicht möglich - er würde nur die Aufnahme im
 * Intervall an- und ausschalten.
 *
 * Quellen: RunCam Device Protocol, Betaflight src/main/io/rcdevice.h,
 *          ArduPilot libraries/AP_Camera/AP_RunCam.{h,cpp},
 *          RunCam Split 4-25 User Manual.
 */

namespace RunCam {

/// Standard-Baudrate des RunCam Device Protocol.
static constexpr uint32_t BAUDRATE = 115200;

/// Startbyte jedes Pakets in beide Richtungen.
static constexpr uint8_t HEADER = 0xCC;

/// Command-IDs (Byte 1 des Pakets).
namespace Cmd {
    static constexpr uint8_t GET_DEVICE_INFO        = 0x00;
    static constexpr uint8_t CAMERA_CONTROL         = 0x01;
    static constexpr uint8_t KEY5_SIMULATION_PRESS  = 0x02;
    static constexpr uint8_t KEY5_SIMULATION_RELEASE= 0x03;
    static constexpr uint8_t KEY5_CONNECTION        = 0x04;
    static constexpr uint8_t REQUEST_FC_ATTITUDE    = 0x50;
}

/// Action-IDs für Cmd::CAMERA_CONTROL (Byte 2 des Pakets).
namespace Action {
    static constexpr uint8_t SIMULATE_WIFI_BTN  = 0x00;
    static constexpr uint8_t SIMULATE_POWER_BTN = 0x01;  ///< Split 4: Aufnahme-Toggle
    static constexpr uint8_t CHANGE_MODE        = 0x02;  ///< Video <-> OSD-Menü
    static constexpr uint8_t START_RECORDING    = 0x03;  ///< von Split 4 NICHT unterstützt
    static constexpr uint8_t STOP_RECORDING     = 0x04;  ///< von Split 4 NICHT unterstützt
}

/// Feature-Bitmaske aus der GET_DEVICE_INFO-Antwort (little endian).
namespace Feature {
    static constexpr uint16_t SIMULATE_POWER_BUTTON  = 1u << 0;
    static constexpr uint16_t SIMULATE_WIFI_BUTTON   = 1u << 1;
    static constexpr uint16_t CHANGE_MODE            = 1u << 2;
    static constexpr uint16_t SIMULATE_5_KEY_OSD     = 1u << 3;
    static constexpr uint16_t DEVICE_SETTINGS_ACCESS = 1u << 4;
    static constexpr uint16_t DISPLAY_PORT           = 1u << 5;
    static constexpr uint16_t START_RECORDING        = 1u << 6;
    static constexpr uint16_t STOP_RECORDING         = 1u << 7;
    static constexpr uint16_t CMS_MENU               = 1u << 8;
    static constexpr uint16_t FC_ATTITUDE            = 1u << 9;
}

/// Antwortlänge von GET_DEVICE_INFO: 0xCC, Version, Feature_lo, Feature_hi, CRC8.
static constexpr size_t DEVICE_INFO_RESPONSE_LENGTH = 5;

/// Längstes von uns gesendetes Paket: Header + Command + Action + CRC.
static constexpr size_t MAX_TX_PACKET_LENGTH = 4;

/// CRC8/DVB-S2, Polynom 0xD5 - identisch zu Betaflight/ArduPilot crc8_dvb_s2().
inline uint8_t crc8DvbS2(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; ++i) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                           : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

/// CRC8/DVB-S2 über einen Puffer, Startwert 0x00.
inline uint8_t crc8DvbS2Buffer(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = crc8DvbS2(crc, data[i]);
    }
    return crc;
}

}  // namespace RunCam

/**
 * @brief Kameramodell - bestimmt, WIE die Aufnahme gesteuert wird.
 *
 * Bewusst konfiguriert statt automatisch erkannt: die Split 4 meldet die
 * Feature-Bits START/STOP_RECORDING, ignoriert die Kommandos aber. Eine
 * Erkennung über die Feature-Bits würde also die falsche Strategie wählen.
 */
enum class RunCamModel : uint8_t {
    SPLIT_4,        ///< Split 4 / Split 4K: Aufnahme-Toggle via SIMULATE_POWER_BTN
    SPLIT_LEGACY,   ///< Split / Split Micro: echtes START_RECORDING / STOP_RECORDING
};

/**
 * @brief Automatisches Triggern der Aufnahme.
 *
 * Ein Snapshot-/Intervallmodus existiert bewusst nicht - das RunCam-Protokoll
 * kennt kein Foto-Kommando (siehe Dateikopf).
 */
enum class CameraTriggerMode : uint8_t {
    MANUAL_ONLY,     ///< Nur explizite Aufrufe von startRecording()/stopRecording()
    RECORD_ON_BOOT,  ///< Sobald die Kamera bereit ist einmalig Aufnahme starten
};

/// Interner Zustand des Boot-/Handshake-Ablaufs.
enum class CameraState : uint8_t {
    UNCONFIGURED,  ///< Kamera deaktiviert oder kein Bus, HAL tut nichts
    BOOTING,       ///< Wartet boot_delay_ms nach dem Einschalten ab
    DETECTING,     ///< Handshake läuft (GET_DEVICE_INFO)
    READY,         ///< Kommandos werden gesendet
};

/// Konfiguration einer einzelnen Kamera (siehe camera_config.hpp).
struct CameraConfig {
    uint8_t camera_id;
    uint8_t mux_channel;      ///< 0..3, siehe CameraBus
    bool enabled;             ///< false = nicht bestückt, HAL bleibt inaktiv
    RunCamModel model;
    CameraTriggerMode mode;
    uint32_t boot_delay_ms;   ///< Wartezeit nach Power-On, bevor der UART antwortet
};

class CameraHAL {
public:
    /// Wartezeit nach Power-On, bis die Kamera auf dem UART antwortet.
    static constexpr uint32_t DEFAULT_BOOT_DELAY_MS = 7000;
    /// Abstand zwischen zwei GET_DEVICE_INFO-Versuchen.
    static constexpr uint32_t PROBE_INTERVAL_MS = 1000;
    /// Anzahl Handshake-Versuche, danach wird "blind" weitergesendet.
    static constexpr uint8_t MAX_PROBE_ATTEMPTS = 5;
    /// Mindestabstand zwischen zwei Steuerkommandos (schützt vor Doppel-Toggle).
    static constexpr uint32_t MIN_CONTROL_INTERVAL_MS = 500;
    /// Wartezeit auf die Handshake-Antwort. 5 Byte brauchen bei 115200 Baud
    /// gut 0,4 ms; die Kamera antwortet innerhalb weniger Millisekunden.
    static constexpr uint32_t RESPONSE_TIMEOUT_MS = 50;

    CameraHAL(const CameraConfig& config, CameraBus* bus);

    /// Startet den Boot-Timer. false, wenn die Kamera deaktiviert ist oder
    /// kein nutzbarer Bus vorliegt. Der UART selbst wird von CameraBus geöffnet.
    bool init();

    /// Muss zyklisch aufgerufen werden: Boot-Delay, Handshake, Auto-Trigger.
    void update(uint32_t now_ms);

    /// Startet die Aufnahme. Idempotent - tut nichts, wenn bereits aufgenommen wird.
    bool startRecording();
    /// Stoppt die Aufnahme. Idempotent.
    bool stopRecording();
    /// Schaltet zwischen Video- und OSD-Menü-Modus um (Action CHANGE_MODE).
    bool changeMode();

    /// Führt den Device-Info-Handshake sofort aus (blockierend, max.
    /// RESPONSE_TIMEOUT_MS) und aktualisiert isDetected()/getFeatures().
    /// Für Diagnose und den Hardware-Test.
    bool probe();

    /// Sendet ein rohes Kamerakommando inkl. Header und CRC.
    bool sendCommand(uint8_t command, const uint8_t* payload, size_t payload_length);
    /// Kurzform für Cmd::CAMERA_CONTROL mit einer Action.
    bool sendControlAction(uint8_t action);

    uint8_t getCameraID() const { return camera_id_; }
    uint8_t getMuxChannel() const { return channel_; }
    /// true, wenn die Kamera aktiviert ist und ein Bus vorliegt (sagt nichts
    /// darüber aus, ob tatsächlich eine Kamera angeschlossen ist).
    bool isPresent() const { return state_ != CameraState::UNCONFIGURED; }
    /// true, wenn die Kamera auf GET_DEVICE_INFO geantwortet hat (CRC geprüft).
    bool isDetected() const { return device_detected_; }
    /// Von der Firmware mitgeführter Aufnahmezustand (die Kamera meldet ihn nicht).
    bool isRecording() const { return recording_; }
    CameraState getState() const { return state_; }
    /// Protokollversion aus der GET_DEVICE_INFO-Antwort, 0 wenn unbekannt.
    uint8_t getProtocolVersion() const { return protocol_version_; }
    /// Feature-Bitmaske aus der GET_DEVICE_INFO-Antwort, 0 wenn unbekannt.
    uint16_t getFeatures() const { return features_; }

    /// Sendet GET_DEVICE_INFO und wartet blockierend (max. RESPONSE_TIMEOUT_MS)
    /// auf die Antwort - der Mux-Kanal liegt nur währenddessen an.
    bool performHandshake();

    uint8_t camera_id_;
    uint8_t channel_;
    CameraBus* bus_;
    RunCamModel model_;
    CameraTriggerMode mode_;
    uint32_t boot_delay_ms_;
    bool enabled_;

    CameraState state_ = CameraState::UNCONFIGURED;
    uint32_t init_ms_ = 0;
    uint32_t last_probe_ms_ = 0;
    uint8_t probe_attempts_ = 0;

    bool device_detected_ = false;
    uint8_t protocol_version_ = 0;
    uint16_t features_ = 0;

    bool recording_ = false;
    bool auto_record_done_ = false;
    uint32_t last_control_ms_ = 0;
};
