#pragma once

#include <cstdint>
#include "mission_state.hpp"

/**
 * @file state_machine.hpp
 * @brief MAGGIE Missions-Zustandsmaschine (reduziertes Gerüst)
 *
 * Flugablauf:
 *   PRE_LAUNCH --SODS--> ARMED --LO--> ASCENT --SOE--> EXPERIMENT --Timer--> SAFE
 *   jederzeit --Abort--> ABORT
 *
 * Bodentest (Review/Integration, siehe enterTest/exitTest):
 *   PRE_LAUNCH --TC TEST_ENTER--> TEST --TC TEST_EXIT--> PRE_LAUNCH
 *
 * TEST ist der einzige Zustand, in dem Aktor-Telecommands (Motor/HDRM)
 * ausgeführt werden - System::handleUplink() weist sie sonst ab. Kommt im
 * TEST-Zustand ein echtes SODS-Signal, gewinnt der Flug: die Maschine geht
 * nach ARMED und der Testbetrieb ist beendet.
 *
 * TODO: Experimentsequenz in handleExperiment() füllen, sobald der Arm da ist.
 */

/// Sensordaten pro update()-Aufruf. System liest sie aus den HALs - die
/// State Machine kennt keine HAL-Objekte direkt.
struct StateMachineInputs {
    // REXUS-Signale, siehe REXUSHAL
    bool sods = false;   ///< Start of Data Storage (~T-600s) -> ARMED
    bool l0   = false;   ///< Liftoff (T=0) -> ASCENT
    bool soe  = false;   ///< Start of Experiment -> EXPERIMENT

    /// Last am Greifer. TODO: Quelle festlegen (Force Sensor 2 oder eigene Wegezelle).
    float force_load_n = 0.0f;

    /// Manueller Abbruch per Telecommand (UplinkOpcode::ABORT).
    bool operator_abort = false;
};

class StateMachine {
public:
    StateMachine() = default;

    /// Setzt Startzustand (PRE_LAUNCH) und alle Timer zurück.
    void init();

    /// Einmal pro System::run()-Durchlauf aufrufen.
    void update(uint32_t now_ms, const StateMachineInputs& in);

    MissionState getState() const { return state_; }

    /// Klartextname des aktuellen Zustands (fürs Log).
    static const char* toString(MissionState state);

    /**
     * @brief Bodentest-Modus betreten (Telecommand TEST_ENTER).
     * Nur aus PRE_LAUNCH erlaubt - im Flug bleibt der Zustand unverändert.
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

    /// Zeit seit Liftoff in ms (0 solange LO noch nicht kam).
    uint32_t getMissionTimeMs(uint32_t now_ms) const {
        return (t_lo_ms_ == 0) ? 0 : (now_ms - t_lo_ms_);
    }

private:
    MissionState state_ = MissionState::PRE_LAUNCH;

    uint32_t t_lo_ms_  = 0;   ///< Zeitpunkt LO=HIGH (T=0); 0 = noch nicht gestartet
    uint32_t t_soe_ms_ = 0;   ///< Zeitpunkt SOE=HIGH (Start Experimentfenster)

    void transitionTo(MissionState next, uint32_t now_ms);
    bool checkAbort(const StateMachineInputs& in, uint32_t now_ms);

    void handlePreLaunch(const StateMachineInputs& in, uint32_t now_ms);
    void handleArmed(const StateMachineInputs& in, uint32_t now_ms);
    void handleAscent(const StateMachineInputs& in, uint32_t now_ms);
    void handleExperiment(uint32_t now_ms);
    void handleTest(const StateMachineInputs& in, uint32_t now_ms);
};
