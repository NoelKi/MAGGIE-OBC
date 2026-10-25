#pragma once

#include <cstdint>
#include "mission_state.hpp"

/**
 * @file state_machine.hpp
 * @brief MAGGIE Zustandsmaschine - schlanker Testaufbau
 *
 * Ablauf:
 *   PRE_LAUNCH --TC TEST_ENTER--> TEST --TC TEST_EXIT--> PRE_LAUNCH
 *   aus beiden --TC ABORT--> ABORT --TC STATE_RESET--> PRE_LAUNCH
 *
 * Neustart (Watchdog, Stromausfall): System speichert jeden Zustandswechsel
 * nichtfluechtig (PersistentState) und startet ueber stateAfterReboot() wieder
 * im richtigen Zustand. ABORT bleibt ueber jeden Neustart erhalten.
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

    /// Setzt den Startzustand - nach einem Neustart den aus stateAfterReboot().
    void init(MissionState start = MissionState::PRE_LAUNCH);

    /**
     * @brief In welchem Zustand nach einem Neustart weitermachen?
     *
     * EINZIGE Stelle, die das entscheidet - kommen die Flugzustaende zurueck,
     * wird hier festgelegt, welche davon fortgesetzt werden.
     *
     * @param stored gespeicherter Zustand (Rohwert aus dem EEPROM)
     * @param valid  false, wenn nie gespeichert oder beschaedigt
     */
    static MissionState stateAfterReboot(uint8_t stored, bool valid);

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

    /**
     * @brief ABORT verlassen (Telecommand STATE_RESET) -> PRE_LAUNCH.
     * Nur aus ABORT - ABORT ist sonst endgueltig, auch ueber Neustarts.
     * @return true, wenn der Wechsel stattgefunden hat
     */
    bool resetFromAbort(uint32_t now_ms);

    /// true, solange Aktor-Telecommands ausgeführt werden dürfen.
    bool actuatorsUnlocked() const { return state_ == MissionState::TEST; }

private:
    MissionState state_ = MissionState::PRE_LAUNCH;

    void transitionTo(MissionState next, uint32_t now_ms);
};
