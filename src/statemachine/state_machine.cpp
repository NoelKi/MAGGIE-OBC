#include "statemachine/state_machine.hpp"
#include <Arduino.h>

namespace {

/// Zustände, in denen der Arm Kontakt zum Target hat und das Force-Limit
/// überhaupt physikalisch sinnvoll ist (siehe checkGlobalAbort).
bool isDockingState(MissionState s) {
    return s == MissionState::APPROACH_T1 || s == MissionState::SUCCESS_T1 ||
           s == MissionState::APPROACH_T2 || s == MissionState::SUCCESS_T2;
}

const char* toString(MissionState s) {
    switch (s) {
        case MissionState::PRE_LAUNCH:   return "PRE_LAUNCH";
        case MissionState::ARMED:        return "ARMED";
        case MissionState::ASCENT:       return "ASCENT";
        case MissionState::MG_DETECT:    return "MG_DETECT";
        case MissionState::MG_CONFIRM:   return "MG_CONFIRM";
        case MissionState::WAIT_FFU:     return "WAIT_FFU";
        case MissionState::HDRMS_OPEN:   return "HDRMS_OPEN";
        case MissionState::ARM_DEPLOY:   return "ARM_DEPLOY";
        case MissionState::APPROACH_T1:  return "APPROACH_T1";
        case MissionState::SUCCESS_T1:   return "SUCCESS_T1";
        case MissionState::APPROACH_T2:  return "APPROACH_T2";
        case MissionState::SUCCESS_T2:   return "SUCCESS_T2";
        case MissionState::ARM_STOW:     return "ARM_STOW";
        case MissionState::SUCCESS_STOW: return "SUCCESS_STOW";
        case MissionState::HDRM_CLOSE:   return "HDRM_CLOSE";
        case MissionState::NET_DEPLOY:   return "NET_DEPLOY";
        case MissionState::SAFE:         return "SAFE";
        case MissionState::ABORT:        return "ABORT";
    }
    return "UNKNOWN";
}

}  // namespace

void StateMachine::init() {
    state_ = MissionState::PRE_LAUNCH;
    mg_source_ = MgSource::NONE;
    t_lo_ms_ = 0;
    t_ug_ms_ = 0;
    mg_confirm_enter_ms_ = 0;
    imu_below_thr_since_ms_ = 0;
    state_enter_ms_ = 0;
}

void StateMachine::transitionTo(MissionState next, uint32_t now_ms) {
    if (next == state_) return;
    Serial.printf("INFO  [StateMachine]: %s -> %s\n", toString(state_), toString(next));
    state_ = next;
    state_enter_ms_ = now_ms;
}

void StateMachine::update(uint32_t now_ms, const StateMachineInputs& in) {
    if (checkGlobalAbort(in, now_ms)) return;

    switch (state_) {
        case MissionState::PRE_LAUNCH:   handlePreLaunch(in, now_ms); break;
        case MissionState::ARMED:        handleArmed(in, now_ms); break;
        case MissionState::ASCENT:       handleAscent(in, now_ms); break;
        case MissionState::MG_DETECT:    handleMgDetect(in, now_ms); break;
        case MissionState::MG_CONFIRM:   handleMgConfirm(in, now_ms); break;
        case MissionState::WAIT_FFU:     handleWaitFfu(now_ms); break;
        case MissionState::HDRMS_OPEN:   handleHdrmsOpen(in, now_ms); break;
        case MissionState::ARM_DEPLOY:   handleArmDeploy(in, now_ms); break;
        case MissionState::APPROACH_T1:  handleApproachT1(in, now_ms); break;
        case MissionState::SUCCESS_T1:   handleSuccessT1(in, now_ms); break;
        case MissionState::APPROACH_T2:  handleApproachT2(in, now_ms); break;
        case MissionState::SUCCESS_T2:   handleSuccessT2(in, now_ms); break;
        case MissionState::ARM_STOW:     handleArmStow(in, now_ms); break;
        case MissionState::SUCCESS_STOW: handleSuccessStow(in, now_ms); break;
        case MissionState::HDRM_CLOSE:   handleHdrmClose(in, now_ms); break;
        case MissionState::NET_DEPLOY:   handleNetDeploy(in, now_ms); break;
        case MissionState::SAFE:         handleSafe(); break;
        case MissionState::ABORT:        handleAbort(); break;
    }
}

bool StateMachine::checkGlobalAbort(const StateMachineInputs& in, uint32_t now_ms) {
    if (state_ == MissionState::ABORT || state_ == MissionState::SAFE) return false;

    if (in.watchdog_timeout || in.power_brownout || in.operator_abort) {
        transitionTo(MissionState::ABORT, now_ms);
        return true;
    }

    // F_load > F_max im Notiz-Block als
    // globalen Trigger, hat aber zusätzlich explizite Force-Abort-Pfeile NUR an
    // APPROACH_T1/T2 und SUCCESS_T1/T2. Global geprüft würden die Aufstiegs-
    // lasten (Boost, mehrere g) das 5-N-Limit sofort reißen und die Mission
    // schon im ASCENT abbrechen. Daher hier bewusst nur in den Docking-Zuständen.
    if (isDockingState(state_) && in.force_load_n > MissionConfig::F_MAX_N) {
        transitionTo(MissionState::ABORT, now_ms);
        return true;
    }
    return false;
}

bool StateMachine::imuBelowThreshold(const StateMachineInputs& in, uint32_t now_ms) {
    if (in.imu_accel_magnitude_g < MissionConfig::IMU_G_THR) {
        if (imu_below_thr_since_ms_ == 0) imu_below_thr_since_ms_ = now_ms;
        return (now_ms - imu_below_thr_since_ms_) >= MissionConfig::IMU_T_THR_MS;
    }
    imu_below_thr_since_ms_ = 0;
    return false;
}

// ---------------------------------------------------------------------------
// Pre-Flight / Flight - vollständig implementiert (nur Timer + REXUS-/IMU-
// Signale nötig, siehe docs/maggie_state_machine.puml)
// ---------------------------------------------------------------------------

void StateMachine::handlePreLaunch(const StateMachineInputs& in, uint32_t now_ms) {
    if (in.sods) {
        transitionTo(MissionState::ARMED, now_ms);
    }
}

void StateMachine::handleArmed(const StateMachineInputs& in, uint32_t now_ms) {
    if (in.l0) {
        t_lo_ms_ = now_ms;
        transitionTo(MissionState::ASCENT, now_ms);
    }
}

void StateMachine::handleAscent(const StateMachineInputs& in, uint32_t now_ms) {
    (void)in;
    if (now_ms - t_lo_ms_ >= MissionConfig::T_MG_EARLIEST_MS) {
        transitionTo(MissionState::MG_DETECT, now_ms);
    }
}

void StateMachine::handleMgDetect(const StateMachineInputs& in, uint32_t now_ms) {
    if (now_ms - t_lo_ms_ > MissionConfig::T_MG_ABORT_MS) {
        transitionTo(MissionState::ABORT, now_ms);
        return;
    }

    if (in.soe) {
        mg_source_ = MgSource::SOE_ONLY;
        t_ug_ms_ = now_ms;
        mg_confirm_enter_ms_ = now_ms;
        transitionTo(MissionState::MG_CONFIRM, now_ms);
        return;
    }

    if (imuBelowThreshold(in, now_ms)) {
        mg_source_ = MgSource::IMU_ONLY;
        // t_µg ist der Zeitpunkt des Schwellen-Übertritts, NICHT der Zeitpunkt,
        // an dem die Debounce-Dauer IMU_T_THR_MS abgelaufen ist.
        t_ug_ms_ = imu_below_thr_since_ms_;
        mg_confirm_enter_ms_ = now_ms;
        transitionTo(MissionState::MG_CONFIRM, now_ms);
    }
}

void StateMachine::handleMgConfirm(const StateMachineInputs& in, uint32_t now_ms) {
    const bool second_source_is_soe = (mg_source_ == MgSource::IMU_ONLY) && in.soe;
    const bool second_source_is_imu = (mg_source_ == MgSource::SOE_ONLY) && imuBelowThreshold(in, now_ms);

    if (second_source_is_soe || second_source_is_imu) {
        mg_source_ = MgSource::BOTH;
        // Diagramm: "t_µg := t_IMU (präziser)" - also der Zeitpunkt des
        // Schwellen-Übertritts, nicht der der Bestätigung.
        if (second_source_is_imu) t_ug_ms_ = imu_below_thr_since_ms_;
        transitionTo(MissionState::WAIT_FFU, now_ms);
        return;
    }

    if (now_ms - mg_confirm_enter_ms_ > MissionConfig::T_XCHECK_MS) {
        mg_source_ = MgSource::DEGRADED;
        transitionTo(MissionState::WAIT_FFU, now_ms);
    }
}

void StateMachine::handleWaitFfu(uint32_t now_ms) {
    if (now_ms - t_ug_ms_ >= MissionConfig::T_FFU_MS) {
        transitionTo(MissionState::HDRMS_OPEN, now_ms);
    }
}

// ---------------------------------------------------------------------------
// Experiment / Docking / Safing - TODO: Arm-Kinematik, HDRM- & Netz-Schalter,
// Trajektorie aus CSV übernehmen und einbinden
// ---------------------------------------------------------------------------

void StateMachine::handleHdrmsOpen(const StateMachineInputs& in, uint32_t now_ms)   { (void)in; (void)now_ms; }
void StateMachine::handleArmDeploy(const StateMachineInputs& in, uint32_t now_ms)   { (void)in; (void)now_ms; }
void StateMachine::handleApproachT1(const StateMachineInputs& in, uint32_t now_ms)  { (void)in; (void)now_ms; }
void StateMachine::handleSuccessT1(const StateMachineInputs& in, uint32_t now_ms)   { (void)in; (void)now_ms; }
void StateMachine::handleApproachT2(const StateMachineInputs& in, uint32_t now_ms)  { (void)in; (void)now_ms; }
void StateMachine::handleSuccessT2(const StateMachineInputs& in, uint32_t now_ms)   { (void)in; (void)now_ms; }
void StateMachine::handleArmStow(const StateMachineInputs& in, uint32_t now_ms)     { (void)in; (void)now_ms; }
void StateMachine::handleSuccessStow(const StateMachineInputs& in, uint32_t now_ms) { (void)in; (void)now_ms; }
void StateMachine::handleHdrmClose(const StateMachineInputs& in, uint32_t now_ms)   { (void)in; (void)now_ms; }
void StateMachine::handleNetDeploy(const StateMachineInputs& in, uint32_t now_ms)   { (void)in; (void)now_ms; }
void StateMachine::handleSafe() {}
void StateMachine::handleAbort() {}
