#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file camera_hal.hpp
 * @brief 4x RunCam Split 4 V2 ueber EINE UART und einen CD4052B-Mux
 *
 * Hardware (Schaltplan "Camera Interface", U15):
 *
 *   Teensy Pin 20 (TX5) ── Cam1_Rx ── Mux Y ──► Y0..Y3 ── RX der Kamera 1..4
 *   Teensy Pin 21 (RX5) ◄─ Cam1_Tx ── Mux X ◄── X0..X3 ── TX der Kamera 1..4
 *   Teensy Pin 19 = Camdir1 = Mux A,  Pin 22 = Camdir2 = Mux B
 *   INH fest auf GND (Mux immer aktiv), VDD 3,3 V, VEE/VSS GND.
 *
 *   B A | Kanal | Kamera
 *   0 0 |  X0/Y0 | 1
 *   0 1 |  X1/Y1 | 2
 *   1 0 |  X2/Y2 | 3
 *   1 1 |  X3/Y3 | 4
 *
 * Es spricht also immer genau EINE Kamera. CameraHAL arbeitet die Auftraege
 * nacheinander ab und schaltet den Mux nur um, wenn nichts mehr unterwegs ist.
 *
 * ---------------------------------------------------------------------------
 * Protokoll: RunCam Device Protocol (wie Betaflight runcam_device.c)
 * ---------------------------------------------------------------------------
 *   Anfrage:  0xCC | Kommando | Parameter... | CRC
 *   CRC:      CRC-8/DVB-S2 (Polynom 0xD5, Start 0) ueber alle Bytes davor
 *
 *   GET_DEVICE_INFO (0x00)  -> Antwort 0xCC | Version | Features LE (2) | CRC
 *   CAMERA_CONTROL  (0x01)  + Aktion, KEINE Antwort:
 *       0x03 START_RECORDING, 0x04 STOP_RECORDING
 *       (0x01 = Power-Taste simulieren - schaltet um, wird hier NIE benutzt:
 *        ohne Rueckmeldung wuesste der OBC nicht, ob er gerade startet oder
 *        stoppt.)
 *
 * Die Kamera meldet nicht, ob sie aufnimmt. "Antwortet" heisst nur: sie ist
 * an, die UART-Strecke samt Mux-Kanal funktioniert. Ob START/STOP unterstuetzt
 * werden, steht in den Feature-Bits (CAM_FEATURE_*).
 *
 * Am echten Aufbau geprueft (19.11.2026):
 *  - Alle 4 Kameras antworten mit 115200 Baud, Protokoll v1, Features 0x0077
 *    (Power/WiFi/Mode-Taste, Bit 4, Bit 5, START_RECORDING - KEIN
 *    STOP_RECORDING). STOP per Telecommand ist deshalb vermutlich wirkungslos.
 *  - START_RECORDING an eine laufende Aufnahme: LED blinkt weiter langsam
 *    (Aufnahme laeuft durch). Gegenprobe auf der SD-Karte (eine Datei ohne
 *    Luecke) steht noch aus.
 *  - RX/TX waren an den Kamerasteckern vertauscht und wurden umgeloetet.
 *  - Kamera 4 braucht teils 1,5-3 min, bis sie antwortet (Startproblem der
 *    Kamera/Versorgung, nicht des OBC - der fragt einfach weiter).
 *  - Einstellungen (Aufloesung, Framerate, Dateilaenge) werden manuell im
 *    Kamera-Menue mit Monitor gesetzt, nicht per UART.
 *
 * Timing: update() blockiert nie. Ein Auftrag dauert ~10 ms (Steuerbefehl)
 * bzw. bis CAM_RESPONSE_TIMEOUT_MS (Abfrage).
 */

static constexpr uint8_t CAM_COUNT = 4;

static constexpr uint8_t CAM_HEADER = 0xCC;

/// Feature-Bits aus GET_DEVICE_INFO (ArduPilot AP_RunCam.h, RCDEVICE_PROTOCOL_FEATURE_*).
/// Bit 4/5 belegt Betaflight anders (dort ohne Bit 4, Bit 4 = DISPLAY_PORT).
static constexpr uint16_t CAM_FEATURE_POWER_BUTTON    = 1u << 0;
static constexpr uint16_t CAM_FEATURE_WIFI_BUTTON     = 1u << 1;
static constexpr uint16_t CAM_FEATURE_CHANGE_MODE     = 1u << 2;
static constexpr uint16_t CAM_FEATURE_5KEY_OSD        = 1u << 3;   ///< Kamera-Menue per 5-Tasten-Simulation
static constexpr uint16_t CAM_FEATURE_SETTINGS_ACCESS = 1u << 4;   ///< Einstellungen per UART lesen/schreiben
static constexpr uint16_t CAM_FEATURE_DISPLAY_PORT    = 1u << 5;
static constexpr uint16_t CAM_FEATURE_START_RECORDING = 1u << 6;
static constexpr uint16_t CAM_FEATURE_STOP_RECORDING  = 1u << 7;

/// Was der OBC an eine Kamera schicken kann. Die Zahlen sind das Low-Byte
/// des Telecommands CAMERA (uplink_hal.hpp) - FEST.
enum class CamAction : uint8_t {
    NONE  = 0,
    START = 1,   ///< CAMERA_CONTROL START_RECORDING
    STOP  = 2,   ///< CAMERA_CONTROL STOP_RECORDING
    PROBE = 3,   ///< GET_DEVICE_INFO - antwortet die Kamera?
};

struct CameraStatus {
    bool     answered_ever   = false;  ///< hat seit dem Start mindestens einmal geantwortet
    uint32_t last_answer_ms  = 0;      ///< millis() der letzten gueltigen Antwort
    uint8_t  protocol        = 0;      ///< Protokollversion aus GET_DEVICE_INFO
    uint16_t features        = 0;      ///< CAM_FEATURE_*
    CamAction last_cmd       = CamAction::NONE;  ///< zuletzt gesendetes START/STOP
    uint16_t probes_failed   = 0;      ///< Abfragen ohne (gueltige) Antwort
    // Diagnose (seit dem Start): kommt ueberhaupt etwas zurueck?
    bool     rx_any          = false;  ///< irgendein Byte auf eine Abfrage empfangen
    bool     rx_garbage      = false;  ///< Bytes empfangen, aber keine gueltige Antwort
                                       ///< (Baudrate? Protokoll? Stoerung?)
};

class CameraHAL {
public:
    /// Pause, nachdem der Mux umgeschaltet hat (CD4052B schaltet in < 1 us,
    /// die Kamera-Leitungen brauchen ein wenig, um sich zu beruhigen).
    static constexpr uint32_t CAM_SETTLE_MS           = 2;
    /// Wartezeit auf die Antwort von GET_DEVICE_INFO.
    static constexpr uint32_t CAM_RESPONSE_TIMEOUT_MS = 300;
    /// Pause nach einem Steuerbefehl, bevor der Mux umschalten darf - die
    /// Bytes muessen die UART-FIFO verlassen haben (4 Byte = 0,35 ms).
    static constexpr uint32_t CAM_TX_GAP_MS           = 5;
    /// Jede Kamera wird reihum abgefragt, alle CAM_PROBE_PERIOD_MS eine.
    static constexpr uint32_t CAM_PROBE_PERIOD_MS     = 1000;
    /// Kamera gilt als "antwortet", wenn die letzte Antwort juenger ist.
    static constexpr uint32_t CAM_ALIVE_MS            = 10000;

    CameraHAL(HardwareSerial& serial, uint8_t pin_sel_a, uint8_t pin_sel_b);

    bool init(uint32_t baudrate);

    /**
     * @brief Auftrag fuer eine Kamera vormerken (cam 0..3).
     *
     * START und STOP ersetzen sich gegenseitig (der letzte Wunsch zaehlt),
     * PROBE kommt zusaetzlich. Ausgefuehrt wird in update().
     */
    void request(uint8_t cam, CamAction action);
    /// Auftrag fuer alle vier Kameras.
    void requestAll(CamAction action);

    /// Jeden Loop aufrufen.
    void update(uint32_t now_ms);

    const CameraStatus& status(uint8_t cam) const { return status_[cam < CAM_COUNT ? cam : 0]; }
    bool alive(uint8_t cam, uint32_t now_ms) const;

    /// CRC-8/DVB-S2 (fuer Tests oeffentlich).
    static uint8_t crc8(const uint8_t* data, uint8_t len);

private:
    enum class Phase : uint8_t { IDLE, SETTLE, WAIT_RESPONSE, GAP };

    HardwareSerial& serial_;
    uint8_t  pin_a_, pin_b_;
    bool     initialized_ = false;

    CameraStatus status_[CAM_COUNT];
    CamAction    pending_cmd_[CAM_COUNT] = {};   ///< START/STOP/NONE
    bool         pending_probe_[CAM_COUNT] = {};

    Phase    phase_       = Phase::IDLE;
    uint8_t  channel_     = 0xFF;     ///< aktuell geschalteter Mux-Kanal
    uint8_t  job_cam_     = 0;
    CamAction job_        = CamAction::NONE;
    uint32_t phase_until_ = 0;
    uint8_t  next_cmd_cam_ = 0;       ///< Reihum-Start fuer START/STOP
    uint8_t  next_probe_cam_ = 0;
    uint32_t next_probe_ms_ = 0;

    uint8_t  rx_[8] = {};
    uint8_t  rx_len_ = 0;
    uint16_t job_rx_bytes_ = 0;      ///< Bytes waehrend der laufenden Abfrage

    bool pickJob(uint32_t now_ms);
    void select(uint8_t channel);
    bool sendJob();
    bool readResponse(uint32_t now_ms);
    void drainRx();
};
