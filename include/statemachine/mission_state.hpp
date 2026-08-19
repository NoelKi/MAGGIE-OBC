#pragma once

#include <cstdint>

/**
 * @file mission_state.hpp
 * @brief MAGGIE Betriebszustände - schlanker Testaufbau
 *
 * Bewusst auf das reduziert, was mit der aktuell verbauten Hardware (IMU,
 * HDRM-Motor mit Encoder, Up-/Downlink, REXUS-Signale) auch wirklich
 * durchfahren werden kann:
 *
 *   PRE_LAUNCH  Grundzustand nach dem Reset, Telemetrie laeuft, Aktoren gesperrt
 *   TEST        Bodentest, Motor-Telecommands freigegeben
 *   ABORT       Endzustand, Aktoren aus - nur ein Reset fuehrt heraus
 *
 * Die Zahlenwerte gehen als DATA[0] im SYS/STATE-Downlink über die Leitung
 * (siehe telemetry_hal.hpp) und sind deshalb FEST. Die Lücke 1..4 stammt aus
 * der frueheren Flugsequenz (ARMED/ASCENT/EXPERIMENT/SAFE) und bleibt
 * reserviert: kommen diese Zustände zurück, behalten sie ihre alten Werte und
 * die Bodenstation muss nicht umgelernt werden.
 */

enum class MissionState : uint8_t {
    PRE_LAUNCH = 0,   // Idle: Telemetrie an, Aktoren gesperrt
    // 1..4 reserviert (ARMED / ASCENT / EXPERIMENT / SAFE der Flugsequenz)
    ABORT      = 5,   // Fehlerfall: Aktoren stoppen, Endzustand
    TEST       = 6,   // Bodentest: Aktoren per Telecommand frei, Telemetrie an
};
