#pragma once

#include <cstdint>

/**
 * @file mission_state.hpp
 * @brief MAGGIE REXUS State Machine - reduziertes Gerüst
 *
 * Bewusst auf die Hauptphasen reduziert. Alle Übergänge lassen sich allein
 * aus den REXUS-Signalen (REXUSHAL) und millis() ableiten - es wird keine
 * Hardware vorausgesetzt, die es im Projekt noch nicht gibt.
 */

enum class MissionState : uint8_t {
    PRE_LAUNCH,   // Idle, Selbsttests - wartet auf SODS
    ARMED,        // SODS high: Datenaufzeichnung an, Aktoren safe - wartet auf LO
    ASCENT,       // LO high (T=0): Flug - wartet auf SOE
    EXPERIMENT,   // SOE high: Experimentfenster (HDRM, Arm, Docking)
    SAFE,         // Experiment beendet: Aktoren aus, Telemetrie läuft weiter
    ABORT,        // Fehlerfall: Aktoren stoppen
};

namespace MissionConfig {
    /// Dauer des Experimentfensters ab SOE. Aus dem Diagramm: T_HARD_CUT = t_µg + 100 s.
    static constexpr uint32_t T_EXPERIMENT_MS = 100000;

    /// Force-Limit -> ABORT. Wird NUR im EXPERIMENT geprüft: In ASCENT
    /// würden die Boost-Lasten (mehrere g) das Limit sofort reißen.
    static constexpr float F_MAX_N = 5.0f;
}
