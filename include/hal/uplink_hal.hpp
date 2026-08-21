#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>

/**
 * @file uplink_hal.hpp
 * @brief MAGGIE Uplink Telecommand (UART, Empfang)
 *
 * Empfängt Telecommands über die RX-Hälfte des Downlink-UART
 * (Teensy 4.1: Serial8, Pin 34 RX). Der Server verpackt das Kommando als
 * SDC-Nutzlast in ein RXSM-Telecommand; das RXSM (Test-/Service-Modul) reicht
 * NUR die Nutzbytes über die UART an den OBC weiter. Der OBC muss das
 * 24-Byte-RXSM-Format daher nicht kennen - er parst nur das kompakte 6-Byte-
 * Command-Frame:
 *
 *  Idx | Feld    | Bytes | Beschreibung
 *  ----+---------+-------+---------------------------------------------
 *   0  | START   |  1    | UL_START (0x7E)
 *   1  | OPCODE  |  1    | UplinkOpcode
 *  2-3 | ARG     |  2    | int16 big-endian (MOTOR_ON: Speed -255..+255,
 *      |         |       | MOTOR_TURN: Drehwinkel in Grad)
 *   4  | CRC     |  1    | CRC-8 (poly 0x07) über Bytes 1..3
 *   5  | END     |  1    | UL_END (0x7F)
 *
 * Frame-Größe 6 Bytes -> passt in die max. 15 SDC-Nutzbytes.
 * CRC-8-Polynom identisch zum Downlink (telemetry_hal.cpp).
 */

static constexpr uint8_t UL_START = 0x7E;
static constexpr uint8_t UL_END   = 0x7F;
static constexpr uint8_t UPLINK_FRAME_SIZE = 6;

/**
 * @brief Telecommand-Opcodes.
 *
 * Die Werte gehen über die Leitung und sind FEST - beim Erweitern nur neue
 * Werte vergeben. Gegenstücke: MAGGIE_SERVER/app/routes/command.py.
 *
 * 0x0x = Motor/HDRM (nur im TEST-Zustand ausgeführt, siehe StateMachine)
 * 0x1x = System/Zustandsmaschine
 */
enum class UplinkOpcode : uint8_t {
    MOTOR_OFF       = 0x00,   ///< Motor ausschalten
    MOTOR_ON        = 0x01,   ///< Motor drehen; ARG = Speed -255..+255 (0 = Default)

    // 0x02..0x04 stillgelegt: MOTOR_HALF_TURN / HALF_TURN_FWD / HALF_TURN_REV.
    // Die Positionsregelung ist entfallen, der Encoder ist nur noch Sensor.
    // Werte NICHT neu vergeben - aeltere Server-/Simulatorstaende senden sie
    // moeglicherweise noch.

    MOTOR_ZERO      = 0x05,   ///< Encoder-Zaehler auf 0 setzen (stoppt den Motor)
    MOTOR_TURN      = 0x06,   ///< Drehung um ARG Grad RELATIV, stoppt am Ziel
    MOTOR_GOTO      = 0x07,   ///< Fahrt auf ARG Grad ABSOLUT (bezogen auf die Encoder-Null)

    TEST_ENTER      = 0x10,   ///< Bodentest-Modus betreten (nur aus PRE_LAUNCH)
    TEST_EXIT       = 0x11,   ///< Bodentest-Modus verlassen -> PRE_LAUNCH
    ABORT           = 0x1F,   ///< Abbruch: Aktoren stoppen -> ABORT
};

struct UplinkCommand {
    uint8_t opcode = 0;
    int16_t arg    = 0;
    bool    valid  = false;
};

class UplinkReceiver {
public:
    /**
     * @brief Constructor
     * @param serial Hardware serial port des Uplinks (gemeinsam mit dem
     *               Downlink, z.B. Serial8). serial.begin() wird NICHT
     *               aufgerufen - der Downlink hat den Port bereits geöffnet.
     */
    explicit UplinkReceiver(HardwareSerial& serial);

    /**
     * @brief UART pollen und ein evtl. vollständiges Kommando liefern.
     * @param cmd wird gefüllt, wenn ein gültiges Frame empfangen wurde
     * @return true, wenn ein gültiges Kommando in cmd steht
     */
    bool poll(UplinkCommand& cmd);

    // -----------------------------------------------------------------------
    // Empfangsstatistik - geht per SYS/UPLINK in den Downlink.
    // Einzige Moeglichkeit, am Boden zu unterscheiden, ob ein Telecommand gar
    // nicht ankommt (rxBytes bleibt stehen) oder nur nicht geparst wird
    // (rxBytes steigt, framesOk nicht).
    // -----------------------------------------------------------------------
    uint32_t rxBytes()   const { return rx_bytes_; }    ///< Bytes auf der RX-Leitung
    uint16_t framesOk()  const { return frames_ok_; }   ///< gueltige Command-Frames
    uint16_t framesBad() const { return frames_bad_; }  ///< verworfene Kandidaten
    uint8_t  lastOpcode() const { return last_opcode_; }///< zuletzt akzeptierter Opcode

    /// Pause, ab der die naechsten Bytes als neuer Burst gelten.
    static constexpr uint32_t BURST_GAP_MS = 50;

    /**
     * @brief Erste Bytes des zuletzt empfangenen Bursts.
     *
     * Zeigt die KOPFbytes des Pakets so, wie sie wirklich an der UART
     * ankamen - erwartet waere EB 90 A5 <mcnt> <dest/len> 7E <opcode> 00.
     */
    const uint8_t* burstBytes() const { return burst_; }
    uint8_t        burstLen()   const { return burst_len_; }

private:
    HardwareSerial& serial_;
    uint8_t buf_[UPLINK_FRAME_SIZE];
    uint8_t len_ = 0;

    static constexpr uint8_t BURST_CAPTURE = 8;
    uint8_t  burst_[BURST_CAPTURE] = {};
    uint8_t  burst_len_    = 0;
    uint32_t last_byte_ms_ = 0;

    uint32_t rx_bytes_    = 0;
    uint16_t frames_ok_   = 0;
    uint16_t frames_bad_  = 0;
    uint8_t  last_opcode_ = 0xFF;

    /// Nach einem ungültigen Frame-Kandidaten auf den nächsten Startmarker
    /// im bereits gelesenen Puffer aufsetzen (siehe uplink_hal.cpp).
    void resync();

    /// CRC-8, polynomial 0x07, init 0x00 (identisch zum Downlink)
    static uint8_t crc8(const uint8_t* data, size_t len);
};
