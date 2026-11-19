#pragma once

#include <cstdint>
#include "mission_state.hpp"

/**
 * @file state_machine.hpp
 * @brief MAGGIE Zustandsmaschine - Boden- und Flugablauf
 *
 *   PRE_LAUNCH ──TC TEST_ENTER──► TEST ──TC TEST_EXIT──► PRE_LAUNCH
 *   jeder Zustand ──TC STATE_RESET──► PRE_LAUNCH   (Testwerkzeug)
 *
 *   PRE_LAUNCH / TEST ──SODS──► ARMED ──LO──► ASCENT ──SOE / 70 s──► EXPERIMENT
 *   EXPERIMENT ──HDRMs offen──► ROBOT_ARM ──t0+115 s──► DESCENT
 *
 * REXUS-Signale sind Wahrheit: Ein spaeteres Signal schliesst die frueheren
 * ein. LO fuehrt aus JEDEM Bodenzustand (auch ohne SODS) nach ASCENT, SOE
 * direkt nach EXPERIMENT. Fehlt ein Signal oder ein Telecommand am Boden,
 * startet das Experiment trotzdem, sobald die Rakete fliegt.
 * Startabbruch auf der Rampe (ARMED, vor LO): STATE_RESET.
 *
 * Die Zustandsmaschine entscheidet nur UEBER Zustaende und Zeiten. Was in
 * einem Zustand passiert (HDRM-Fahrten, Motorsperre) und welche Telecommands
 * gelten, macht System - die Zustandsmaschine kennt keine Hardware.
 *
 * Ab LO gibt es keinen Rueckweg: ASCENT, EXPERIMENT und ROBOT_ARM laufen nur
 * vorwaerts und nur ueber Signale und Zeit.
 */

/// Eingaenge pro update()-Aufruf. System liest sie aus den HALs.
struct StateMachineInputs {
    bool sods = false;              ///< REXUS SODS liegt an (entprellt)
    bool lo   = false;              ///< REXUS LO liegt an (entprellt)
    bool soe  = false;              ///< REXUS SOE liegt an (entprellt)
    bool hdrms_open_done = false;   ///< beide HDRM-Oeffnungsfahrten beendet
};

class StateMachine {
public:
    /**
     * @brief Startzustand setzen.
     * @param start      Zustand aus stateAfterReboot()
     * @param elapsed_ms bereits verstrichene Zeit im Flug (siehe elapsed())
     */
    void init(MissionState start, uint32_t now_ms, uint32_t elapsed_ms);

    /// Einmal pro System::run()-Durchlauf aufrufen.
    void update(uint32_t now_ms, const StateMachineInputs& in);

    MissionState getState() const { return state_; }

    /// Klartextname eines Zustands (fuers Log).
    static const char* toString(MissionState state);

    /**
     * @brief In welchem Zustand nach einem Neustart weitermachen?
     *
     * EINZIGE Stelle, die das entscheidet. Flugzustaende werden fortgesetzt,
     * TEST (und ein alter ABORT-Eintrag) fallen auf PRE_LAUNCH zurueck.
     */
    static MissionState stateAfterReboot(uint8_t stored, bool valid);

    /// Telecommand TEST_ENTER - nur aus PRE_LAUNCH.
    bool enterTest(uint32_t now_ms);
    /// Telecommand TEST_EXIT - nur aus TEST.
    bool exitTest(uint32_t now_ms);
    /// Telecommand STATE_RESET -> PRE_LAUNCH - aus jedem Zustand (Testwerkzeug).
    bool resetToPreLaunch(uint32_t now_ms);

    /// true, solange Motor-Telecommands ausgefuehrt werden duerfen.
    bool actuatorsUnlocked() const { return state_ == MissionState::TEST; }

    /// Zeitbezug: in ASCENT seit LO, ab EXPERIMENT seit t0.
    uint32_t elapsed(uint32_t now_ms) const { return now_ms - base_ms_; }

    /// Bodenzustaende: hier ist kein Flug im Gang.
    static bool isGround(MissionState s) {
        return s == MissionState::PRE_LAUNCH || s == MissionState::TEST;
    }

    /// Zustaende, deren verstrichene Zeit ueber einen Neustart gerettet wird.
    static bool isTimed(MissionState s) {
        return s == MissionState::ASCENT || s == MissionState::EXPERIMENT ||
               s == MissionState::ROBOT_ARM;
    }

private:
    MissionState state_ = MissionState::PRE_LAUNCH;
    uint32_t base_ms_ = 0;   ///< Zeitbezug fuer elapsed()

    void transitionTo(MissionState next, uint32_t now_ms);
};
