#pragma once

#include <cstdint>

/**
 * @file mission_state.hpp
 * @brief MAGGIE Betriebszustaende
 *
 *   Boden:  PRE_LAUNCH, TEST, ABORT
 *   Flug:   ARMED -> ASCENT -> EXPERIMENT -> ROBOT_ARM -> DESCENT
 *           (Ablauf und Zeiten: mission_timeline.hpp)
 *
 * Die Zahlenwerte gehen als DATA[0] im SYS/STATE-Downlink ueber die Leitung
 * und werden im EEPROM gespeichert - sie sind deshalb FEST. Beim Erweitern nur
 * neue Werte vergeben. Gegenstueck am Boden: MISSION_STATES in
 * MAGGIE-SERVER/app/services/downlink_frame_parser.py.
 */

enum class MissionState : uint8_t {
    PRE_LAUNCH = 0,   ///< Grundzustand: Telemetrie an, Aktoren gesperrt, wartet auf SODS
    ARMED      = 1,   ///< SODS erhalten, wartet auf LO (Rampe)
    ASCENT     = 2,   ///< LO erhalten, wartet auf SOE oder ASCENT_MAX_MS
    EXPERIMENT = 3,   ///< t0: HDRMs oeffnen
    DESCENT    = 4,   ///< HDRMs zu, Motoren blockiert - bis Strom aus / STATE_RESET
    ABORT      = 5,   ///< Abbruch am Boden: Aktoren aus
    TEST       = 6,   ///< Bodentest: Aktoren per Telecommand frei
    ROBOT_ARM  = 7,   ///< Roboterarm-Bewegungen (Platzhalter, noch nicht implementiert)
};
