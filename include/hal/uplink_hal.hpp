#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>

/**
 * @file uplink_hal.hpp
 * @brief MAGGIE Uplink Telecommand (UART, Empfang)
 *
 * Empfängt Motor-Telecommands über die RX-Hälfte des Downlink-UART
 * (Teensy 4.1: Serial8, Pin 34 RX). Der Server verpackt das Kommando als
 * SDC-Nutzlast in ein RXSM-Telecommand; das RXSM (Test-/Service-Modul) reicht
 * NUR die Nutzbytes über die UART an den OBC weiter. Der OBC muss das
 * 24-Byte-RXSM-Format daher nicht kennen - er parst nur das kompakte 6-Byte-
 * Motor-Command-Frame:
 *
 *  Idx | Feld    | Bytes | Beschreibung
 *  ----+---------+-------+---------------------------------------------
 *   0  | START   |  1    | UL_START (0x7E)
 *   1  | OPCODE  |  1    | MotorOpcode (OFF/ON/HALF_TURN)
 *  2-3 | ARG     |  2    | int16 big-endian (optionales Argument)
 *   4  | CRC     |  1    | CRC-8 (poly 0x07) über Bytes 1..3
 *   5  | END     |  1    | UL_END (0x7F)
 *
 * Frame-Größe 6 Bytes -> passt in die max. 15 SDC-Nutzbytes.
 * CRC-8-Polynom identisch zum Downlink (telemetry_hal.cpp).
 */

static constexpr uint8_t UL_START = 0x7E;
static constexpr uint8_t UL_END   = 0x7F;
static constexpr uint8_t UPLINK_FRAME_SIZE = 6;

// OPCODE - Motor-Kommandos
enum class MotorOpcode : uint8_t {
    OFF       = 0x00,   ///< Motor ausschalten
    ON        = 0x01,   ///< Motor dauerhaft einschalten
    HALF_TURN = 0x02,   ///< halbe Umdrehung (Closed-Loop)
};

struct MotorCommand {
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
     * @brief UART pollen und ein evtl. vollständiges Motor-Kommando liefern.
     * @param cmd wird gefüllt, wenn ein gültiges Frame empfangen wurde
     * @return true, wenn ein gültiges Kommando in cmd steht
     */
    bool poll(MotorCommand& cmd);

private:
    HardwareSerial& serial_;
    uint8_t buf_[UPLINK_FRAME_SIZE];
    uint8_t len_ = 0;

    /// CRC-8, polynomial 0x07, init 0x00 (identisch zum Downlink)
    static uint8_t crc8(const uint8_t* data, size_t len);
};
