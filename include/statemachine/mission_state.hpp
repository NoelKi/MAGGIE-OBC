#pragma once

#include <cstdint>

/**
 * @file mission_state.hpp
 * @brief MAGGIE REXUS State Machine - reduziertes Gerüst
 *
 * Bewusst auf die Hauptphasen reduziert. Alle Flug-Übergänge lassen sich allein
 * aus den REXUS-Signalen (REXUSHAL) und millis() ableiten - es wird keine
 * Hardware vorausgesetzt, die es im Projekt noch nicht gibt.
 *
 * Die Zahlenwerte gehen als DATA[0] im SYS/STATE-Downlink über die Leitung
 * (siehe telemetry_hal.hpp) und sind deshalb FEST - beim Erweitern nur hinten
 * anhängen, nie umsortieren.
 */

enum class MissionState : uint8_t {
    PRE_LAUNCH = 0,   // Idle, Selbsttests - wartet auf SODS
    ARMED      = 1,   // SODS high: Datenaufzeichnung an, Aktoren safe - wartet auf LO
    ASCENT     = 2,   // LO high (T=0): Flug - wartet auf SOE
    EXPERIMENT = 3,   // SOE high: Experimentfenster (HDRM, Arm, Docking)
    SAFE       = 4,   // Experiment beendet: Aktoren aus, Telemetrie läuft weiter
    ABORT      = 5,   // Fehlerfall: Aktoren stoppen
    TEST       = 6,   // Bodentest: Aktoren per Telecommand frei, Telemetrie an
};

namespace MissionConfig {
    /// Dauer des Experimentfensters ab SOE. Aus dem Diagramm: T_HARD_CUT = t_µg + 100 s.
    static constexpr uint32_t T_EXPERIMENT_MS = 100000;

    /// Force-Limit -> ABORT. Wird NUR im EXPERIMENT geprüft: In ASCENT
    /// würden die Boost-Lasten (mehrere g) das Limit sofort reißen.
    static constexpr float F_MAX_N = 5.0f;
}
