#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>

/**
 * @file arm_hal.hpp
 * @brief UART-Verbindung zur Robot Control Unit (RCU) des Roboterarms (LISA)
 *
 * Der Arm hat eine eigene Steuerplatine (RCU, STM32). Sie setzt die UART-
 * Befehle in CANopen fuer die einzelnen Gelenkmodule um und schickt deren
 * Telemetrie zurueck. Der OBC spricht nur mit der RCU - an Pin 0 (RX1) und
 * 1 (TX1), also Serial1 (siehe pin_config.hpp).
 *
 * Quelle des Protokolls: LISA-ODISEE ICD v0.9,
 * docs/dataSheets/LISA_ODISEE_Interface_Control_Document.pdf, Kapitel 3.3.
 * Referenzimplementierung (Python, nicht veraendern): extern/lisa-if/lisa/.
 *
 * ---------------------------------------------------------------------------
 * Frame (beide Richtungen, Payload little-endian)
 * ---------------------------------------------------------------------------
 *  Idx  | Feld     | Bytes | Beschreibung
 *  -----+----------+-------+------------------------------------------------
 *   0   | SYNC     |  1    | ARM_SYNC (0xAA)
 *   1   | NODE     |  1    | Gelenk (CANopen-Node-ID), 0 = RCU selbst
 *   2   | ID       |  1    | ArmMsg
 *   3   | RESERVED |  1    | 0x00
 *  4..  | PAYLOAD  | 0..16 | Laenge haengt NUR an der ID (keine Laengenangabe)
 *  +4   | CRC      |  4    | CRC-32/MPEG-2 ueber Header + Payload, little-endian
 *
 * Weil das Frame keine Laenge traegt, muss der Empfaenger jede ID kennen.
 * Eine unbekannte ID laesst sich nicht ueberspringen - der Parser verwirft
 * dann nur das Sync-Byte und sucht das naechste (wie lisa-if).
 *
 * ---------------------------------------------------------------------------
 * Gelenkzustand eines Moduls (ICD 3.2)
 * ---------------------------------------------------------------------------
 *   NOT REFERENCED -> (Referenz) -> INHIBITED <-RELEASE/INHIBIT-> READY
 *   READY -> (MOVE_*) -> MOVING -> (Ziel erreicht / STOP) -> READY
 *
 * Ein Modul in INHIBITED ignoriert Fahrbefehle. Vor einem Jog muss also
 * RELEASE kommen. Die Bitbelegung des state-Bytes in JOINT_STATE beschreibt
 * die ICD nur als "SM phase + reference source" - der OBC reicht es roh
 * durch, die Bodenstation zeigt es hexadezimal.
 *
 * ---------------------------------------------------------------------------
 * Timing
 * ---------------------------------------------------------------------------
 * poll() liest nur, was im UART-Puffer liegt, und kehrt sofort zurueck.
 * Senden geht in den TX-Puffer (siehe ARM_TX_EXTRA in system.cpp) und
 * blockiert nicht. Bytes, die waehrend einer gesperrten Interrupt-Phase
 * (Flash-Schreiben, siehe persistent_state.hpp) verloren gehen, fallen als
 * CRC-Fehler auf, der Parser synchronisiert sich am naechsten 0xAA neu.
 */

static constexpr uint8_t ARM_SYNC        = 0xAA;
static constexpr uint8_t ARM_HEADER_SIZE = 4;
static constexpr uint8_t ARM_CRC_SIZE    = 4;
static constexpr uint8_t ARM_MAX_PAYLOAD = 16;
static constexpr uint8_t ARM_MAX_FRAME   = ARM_HEADER_SIZE + ARM_MAX_PAYLOAD + ARM_CRC_SIZE;

/// Node-ID der RCU selbst (RCU-IMU, LSS-Scan, ...).
static constexpr uint8_t ARM_NODE_RCU = 0;

/**
 * Hoechste Gelenk-Node-ID, die der OBC verwaltet. Die ICD legt die Anzahl
 * nicht fest ("Module 1..N"). Nodes darueber werden gezaehlt, aber nicht
 * gespeichert. STOP/INHIBIT "an alle" geht an 1..ARM_MAX_JOINTS.
 */
static constexpr uint8_t ARM_MAX_JOINTS = 8;

/// Ohne gueltiges Frame laenger als das gilt die Verbindung als tot.
static constexpr uint32_t ARM_LINK_TIMEOUT_MS = 1000;

/// Nachrichten-IDs (ICD Tabelle 3 und 9). Werte gehen ueber die Leitung.
enum class ArmMsg : uint8_t {
    // Befehle OBC -> RCU
    STOP                 = 0x01,
    INHIBIT              = 0x02,
    RELEASE              = 0x03,
    PAUSE                = 0x04,
    CONTINUE             = 0x05,
    STEPPER_MOVE_REL     = 0x06,   ///< int32 urad, relativ
    STEPPER_MOVE_ABS     = 0x07,   ///< int32 urad, absolut
    STEPPER_MOVE_STEPS   = 0x08,
    RESET                = 0x09,
    STEPPER_SET_VELOCITY = 0x0A,   ///< uint32 urad/s + uint32 urad/s^2 (RCU-Standard 6 Grad/s / 2 Grad/s^2)
    SET_HOME             = 0x0B,
    INITIATE_LSS_SCAN    = 0x0C,
    SET_NODE_ID          = 0x0D,
    BLINK                = 0x0E,

    // Telemetrie RCU -> OBC
    JOINT_STATE          = 0x14,   ///< state(uint8) + angle(int32 urad)
    IMU_DATA             = 0x15,   ///< Gelenk-IMU (16 Byte)
    VOLTAGE_DATA         = 0x16,   ///< bus_mv(uint16) + stepper_mv(uint16)
    TEMPERATURE_DATA     = 0x17,   ///< mcu(int16) + imu(int16)
    REFERENCE_ANGLE_DATA = 0x18,   ///< uint32 urad
    IDENTIFIED           = 0x19,   ///< LSS-Adresse (16 Byte)
    RCU_IMU_DATA         = 0x1A,   ///< 6x int16, RCU-eigene IMU
    RCU_FREEFALL_EVENT   = 0x1B,   ///< ohne Payload
};

/// Art eines Frames fuer den Mithoerer (ArmLink::setTap).
enum class ArmTapKind : uint8_t {
    RX         = 0,   ///< von der RCU empfangen, CRC ok
    TX         = 1,   ///< an die RCU gesendet
    RX_BAD_CRC = 2,   ///< Kandidat mit falscher CRC (Bytes inkl. CRC)
    RX_BAD_ID  = 3,   ///< Kandidat mit unbekannter ID (nur der Kopf)
};

/// Letzter Stand eines Gelenks. age = millis() - updated_ms.
struct ArmJoint {
    // JOINT_STATE
    bool     seen       = false;   ///< je ein JOINT_STATE empfangen
    uint8_t  state      = 0;       ///< roh, siehe Dateikopf
    int32_t  angle_urad = 0;
    uint32_t state_ms   = 0;
    bool     state_new  = false;   ///< seit dem letzten Downlink aktualisiert

    // VOLTAGE_DATA
    uint16_t bus_mv     = 0;
    uint16_t stepper_mv = 0;
    bool     voltage_new = false;

    // TEMPERATURE_DATA (Rohwerte, Skalierung siehe Server)
    int16_t  mcu_temp   = 0;
    int16_t  imu_temp   = 0;
    bool     temp_new   = false;
};

/// RCU-eigene IMU (ICD 3.3.2.7): rohe Counts, +-4 g / +-500 dps.
struct ArmRcuImu {
    int16_t  acc[3]  = {};
    int16_t  gyro[3] = {};
    uint32_t updated_ms = 0;
    bool     fresh   = false;   ///< seit dem letzten Downlink aktualisiert
};

class ArmLink {
public:
    /**
     * @param serial UART zur RCU (Serial1). Geoeffnet wird sie in init().
     */
    explicit ArmLink(HardwareSerial& serial);

    /// UART oeffnen. Ob die RCU antwortet, zeigt erst linkAlive().
    bool init(uint32_t baudrate);

    /// Empfangene Bytes parsen - jeden Loop aufrufen, blockiert nie.
    void poll();

    // -----------------------------------------------------------------------
    // Befehle. Rueckgabe false: TX-Puffer voll, Frame NICHT gesendet.
    // -----------------------------------------------------------------------
    bool sendCommand(uint8_t node, ArmMsg msg);
    bool sendMoveRel(uint8_t node, int32_t angle_urad);
    /// Absolutes Ziel. Waehrend einer Fahrt aendert es nur das Ziel, das
    /// Modul haelt dafuer nicht an (am Gelenk bestaetigt).
    bool sendMoveAbs(uint8_t node, int32_t angle_urad);
    /// Geschwindigkeit und Beschleunigung, Angabe in Grad/s bzw. Grad/s^2.
    /// Auf der Leitung gehen urad/s und urad/s^2 (uint32) - Test 15.11.2026:
    /// ganze Grad/s liessen das Gelenk nur kriechen (state "faehrt", ~0 Grad/s),
    /// urad-Werte fuhren es. Standard der RCU: 6 Grad/s, 2 Grad/s^2.
    bool sendSetVelocity(uint8_t node, float speed_deg_s, float accel_deg_s2);

    /// Freie Bytes im TX-Puffer - zum Pruefen, ob mehrere Frames zusammen
    /// passen (ArmFlightPlayer sendet alle Gelenke oder keins).
    int txFree() const;

    /// STOP bzw. INHIBIT an alle Gelenke 1..ARM_MAX_JOINTS.
    void stopAll();
    void inhibitAll();

    // -----------------------------------------------------------------------
    // Empfangener Stand
    // -----------------------------------------------------------------------
    /// Gelenk 1..ARM_MAX_JOINTS, sonst nullptr.
    ArmJoint*       joint(uint8_t node);
    ArmRcuImu&      rcuImu() { return rcu_imu_; }

    bool     linkAlive(uint32_t now_ms) const;
    uint32_t lastFrameMs()   const { return last_frame_ms_; }
    uint16_t framesOk()      const { return frames_ok_; }
    uint16_t framesBad()     const { return frames_bad_; }   ///< CRC-Fehler + unbekannte IDs
    uint32_t crcErrors()     const { return crc_errors_; }   ///< davon: CRC falsch
    uint32_t unknownIds()    const { return unknown_ids_; }  ///< davon: ID unbekannt
    uint32_t framesOkTotal() const { return frames_ok_total_; }
    uint8_t  lastMsg()       const { return last_msg_; }
    uint8_t  lastNode()      const { return last_node_; }
    uint8_t  freefallCount() const { return freefall_count_; }
    uint16_t txDropped()     const { return tx_dropped_; }

    /// CRC-32/MPEG-2 (Poly 0x04C11DB7, Init 0xFFFFFFFF, keine Reflexion,
    /// kein XorOut). Pruefwert "123456789" -> 0x0376E6E7.
    static uint32_t crc32(const uint8_t* data, size_t len);

    /// Payload-Laenge einer ID, -1 wenn unbekannt.
    static int payloadSize(uint8_t msg_id);

    /**
     * @brief Mithoerer fuer jedes Frame (z.B. die SD-Karte).
     *
     * RX/TX: gueltiges Frame, Kopf + Payload OHNE CRC. RX_BAD_*: verworfener
     * Kandidat mit den Rohbytes (Diagnose). Darf nicht blockieren.
     */
    using FrameTap = void (*)(void* ctx, ArmTapKind kind, const uint8_t* frame, uint8_t len);
    void setTap(FrameTap tap, void* ctx) { tap_ = tap; tap_ctx_ = ctx; }

private:
    FrameTap tap_     = nullptr;
    void*    tap_ctx_ = nullptr;

    HardwareSerial& serial_;
    bool initialized_ = false;

    uint8_t buf_[ARM_MAX_FRAME];
    uint8_t len_ = 0;

    ArmJoint  joints_[ARM_MAX_JOINTS];
    ArmRcuImu rcu_imu_;

    uint32_t last_frame_ms_  = 0;
    bool     ever_received_  = false;
    uint16_t frames_ok_      = 0;
    uint16_t frames_bad_     = 0;
    uint32_t crc_errors_     = 0;
    uint32_t unknown_ids_    = 0;
    uint32_t frames_ok_total_ = 0;
    uint8_t  last_msg_       = 0;
    uint8_t  last_node_      = 0;
    uint8_t  freefall_count_ = 0;
    uint16_t tx_dropped_     = 0;

    bool send(uint8_t node, ArmMsg msg, const uint8_t* payload, uint8_t len);
    void parseBuffer();
    void handleFrame(uint8_t node, uint8_t msg_id, const uint8_t* payload);
    /// n Bytes verwerfen und auf das naechste Sync-Byte dahinter aufsetzen.
    /// drop(1) nach einem ungueltigen Kandidaten = Resync.
    void drop(uint8_t n);
};
