#pragma once

#include <cstdint>
#include "mission_state.hpp"

/**
 * @file state_machine.hpp
 * @brief MAGGIE Missions-Zustandsmaschine (Grundgerüst)
 *  TODO ins System integrieren und fehlende Komponenten implementieren
 */

// Sensordaten, die die Zustandsmaschine pro update()-Aufruf braucht
struct StateMachineInputs {
    // REXUS-Signale, siehe REXUSHAL (aktuell nicht in System verdrahtet)
    bool l0 = false;
    bool soe = false;
    bool sods = false;

    // IMU, siehe IMUHAL::read() -> IMUReading
    float imu_accel_magnitude_g = 1.0f;   ///< |a|; ~1g = Ruhe/Rampe, ~0g = Mikrogravitation

    // TODO: welcher Sensor liefert das genau Force Sensor 2?
    float force_load_n = 0.0f;

    // Globale Abort-Trigger - TODO:
    // reale Quellen existieren im Projekt noch nicht.
    bool watchdog_timeout = false;   // TODO: keine Watchdog-HAL im Projekt
    bool power_brownout = false;     // TODO: keine Spannungsüberwachung im Projekt
    bool operator_abort = false;     // TODO: kein Uplink-Kommando-Parser im Projekt
};

class StateMachine {
public:
    StateMachine() = default;

    // Setzt Startzustand (PRE_LAUNCH) und alle Timer zurück.
    void init();

    // Einmal pro System::run()-Durchlauf aufrufen.
    void update(uint32_t now_ms, const StateMachineInputs& in);

    MissionState getState() const { return state_; }
    MgSource getMgSource() const { return mg_source_; }

private:
    MissionState state_ = MissionState::PRE_LAUNCH;
    MgSource mg_source_ = MgSource::NONE;

    uint32_t t_lo_ms_ = 0;                  // Zeitpunkt LO=HIGH (T=0)
    uint32_t t_ug_ms_ = 0;                  // Zeitpunkt Mikrogravitations-Erkennung (t_µg)
    uint32_t mg_confirm_enter_ms_ = 0;      // Eintrittszeit in MG_CONFIRM (für T_xcheck)
    uint32_t imu_below_thr_since_ms_ = 0;   // 0 = IMU aktuell nicht unter Schwelle
    uint32_t state_enter_ms_ = 0;           // Eintrittszeit in den aktuellen Zustand (für spätere TODO-Timer nutzbar)

    void transitionTo(MissionState next, uint32_t now_ms);
    bool checkGlobalAbort(const StateMachineInputs& in, uint32_t now_ms);
    bool imuBelowThreshold(const StateMachineInputs& in, uint32_t now_ms);

    // Ein Handler pro Zustand 
    void handlePreLaunch(const StateMachineInputs& in, uint32_t now_ms);
    void handleArmed(const StateMachineInputs& in, uint32_t now_ms);
    void handleAscent(const StateMachineInputs& in, uint32_t now_ms);
    void handleMgDetect(const StateMachineInputs& in, uint32_t now_ms);
    void handleMgConfirm(const StateMachineInputs& in, uint32_t now_ms);
    void handleWaitFfu(uint32_t now_ms);

    // TODO: siehe Klassenkommentar oben - Zustände nur angelegt
    void handleHdrmsOpen(const StateMachineInputs& in, uint32_t now_ms);
    void handleArmDeploy(const StateMachineInputs& in, uint32_t now_ms);
    void handleApproachT1(const StateMachineInputs& in, uint32_t now_ms);
    void handleSuccessT1(const StateMachineInputs& in, uint32_t now_ms);
    void handleApproachT2(const StateMachineInputs& in, uint32_t now_ms);
    void handleSuccessT2(const StateMachineInputs& in, uint32_t now_ms);
    void handleArmStow(const StateMachineInputs& in, uint32_t now_ms);
    void handleSuccessStow(const StateMachineInputs& in, uint32_t now_ms);
    void handleHdrmClose(const StateMachineInputs& in, uint32_t now_ms);
    void handleNetDeploy(const StateMachineInputs& in, uint32_t now_ms);
    void handleSafe();
    void handleAbort();
};
