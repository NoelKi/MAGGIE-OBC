#pragma once
#include <Arduino.h>

// PLATZHALTER - noch keine Flug-Trajektorie eingebaut.
// ARM_FLIGHT_VALID = false: Der OBC faehrt den Arm im Flug NICHT und meldet
// "keine Tabelle" (SYS/ARM_FLIGHT, Flag TABLE_OK fehlt).
//
// Die echte Tabelle erzeugt:
//   python3 tools/arm_trajectory.py flight <flug.csv> --joints 1 2 3 ... --nodes 1 2 3 ...
// Danach Firmware bauen und flashen.

static constexpr bool     ARM_FLIGHT_VALID  = false;
static constexpr uint32_t ARM_FLIGHT_DT_MS  = 100;
static constexpr uint8_t  ARM_FLIGHT_JOINTS = 1;
static constexpr uint16_t ARM_FLIGHT_LEN    = 2;
static constexpr uint32_t ARM_FLIGHT_CRC32  = 0x6904BB59u;
static constexpr uint8_t  ARM_FLIGHT_NODES[ARM_FLIGHT_JOINTS] = {1};

static const int32_t ARM_FLIGHT_URAD[ARM_FLIGHT_LEN][ARM_FLIGHT_JOINTS] PROGMEM = {
    {          0},
    {          0},
};
