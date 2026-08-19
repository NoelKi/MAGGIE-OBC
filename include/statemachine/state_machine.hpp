#pragma once

#include <cstdint>
#include "mission_state.hpp"

/**
 * @file state_machine.hpp
 * @brief MAGGIE Zustandsmaschine - schlanker Testaufbau
 *
 * Ablauf:
 *   PRE_LAUNCH --TC TEST_ENTER--> TEST --TC TEST_EXIT--> PRE_LAUNCH
 *   aus beiden --TC ABORT--> ABORT (Endzustand, nur Reset fuehrt heraus)
 *
 * TEST ist der einzige Zustand, in dem Aktor-Telecommands (Motor) ausgeführt
 * werden - System::handleUplink() weist sie sonst ab.
 *
 * Die REXUS-Signale (L0/SOE/SODS) loesen hier bewusst KEINE Uebergaenge mehr
 * aus: sie werden nur noch eingelesen und als Rohpegel mit dem SYS/STATE-Frame
 * heruntergefunkt (siehe System::handleSystemTelemetry). Erst wenn die
 * Flugsequenz zurueckkommt, werden sie wieder zu Eingaengen.
 */

/// Eingaenge pro update()-Aufruf. System liest sie aus den HALs - die
/// State Machine kennt keine HAL-Objekte direkt.
struct StateMachineInputs {
    /// Manueller Abbruch per Telecommand (UplinkOpcode::ABORT).
    bool operator_abort = false;
};

class StateMachine {
public:
    StateMachine() = default;

    /// Setzt den Startzustand (PRE_LAUNCH).
    void init();

    /// Einmal pro System::run()-Durchlauf aufrufen.
    void update(uint32_t now_ms, const StateMachineInputs& in);

    MissionState getState() const { return state_; }

    /// Klartextname des aktuellen Zustands (fürs Log).
    static const char* toString(MissionState state);

    /**
     * @brief Bodentest-Modus betreten (Telecommand TEST_ENTER).
     * Nur aus PRE_LAUNCH erlaubt - aus ABORT fuehrt nur ein Reset heraus.
     * @return true, wenn der Wechsel stattgefunden hat
     */
    bool enterTest(uint32_t now_ms);

    /**
     * @brief Bodentest-Modus verlassen (Telecommand TEST_EXIT) -> PRE_LAUNCH.
     * @return true, wenn der Wechsel stattgefunden hat
     */
    bool exitTest(uint32_t now_ms);

    /// true, solange Aktor-Telecommands ausgeführt werden dürfen.
    bool actuatorsUnlocked() const { return state_ == MissionState::TEST; }

private:
    MissionState state_ = MissionState::PRE_LAUNCH;

    void transitionTo(MissionState next, uint32_t now_ms);
};
