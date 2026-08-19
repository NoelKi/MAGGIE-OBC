#include "statemachine/state_machine.hpp"
#include <Arduino.h>

const char* StateMachine::toString(MissionState s) {
    switch (s) {
        case MissionState::PRE_LAUNCH: return "PRE_LAUNCH";
        case MissionState::ARMED:      return "ARMED";
        case MissionState::ASCENT:     return "ASCENT";
        case MissionState::EXPERIMENT: return "EXPERIMENT";
        case MissionState::SAFE:       return "SAFE";
        case MissionState::ABORT:      return "ABORT";
        case MissionState::TEST:       return "TEST";
    }
    return "UNKNOWN";
}

void StateMachine::init() {
    state_ = MissionState::PRE_LAUNCH;
    t_lo_ms_ = 0;
    t_soe_ms_ = 0;
}

void StateMachine::transitionTo(MissionState next, uint32_t now_ms) {
    if (next == state_) return;
    Serial.printf("INFO  [StateMachine]: %s -> %s (T+%lu ms)\n",
                  toString(state_), toString(next),
                  static_cast<unsigned long>(getMissionTimeMs(now_ms)));
    state_ = next;
}

void StateMachine::update(uint32_t now_ms, const StateMachineInputs& in) {
    if (checkAbort(in, now_ms)) return;

    switch (state_) {
        case MissionState::PRE_LAUNCH: handlePreLaunch(in, now_ms); break;
        case MissionState::ARMED:      handleArmed(in, now_ms); break;
        case MissionState::ASCENT:     handleAscent(in, now_ms); break;
        case MissionState::EXPERIMENT: handleExperiment(now_ms); break;
        case MissionState::TEST:       handleTest(in, now_ms); break;

        // Endzustände - hier passiert nichts mehr.
        case MissionState::SAFE:
        case MissionState::ABORT:
            break;
    }
}

bool StateMachine::enterTest(uint32_t now_ms) {
    // Nur vom Boden aus: sobald SODS gekommen ist, gilt die Flugsequenz.
    if (state_ != MissionState::PRE_LAUNCH) {
        Serial.printf("WARN  [StateMachine]: TEST_ENTER abgelehnt - Zustand ist %s.\n",
                      toString(state_));
        return false;
    }
    transitionTo(MissionState::TEST, now_ms);
    return true;
}

bool StateMachine::exitTest(uint32_t now_ms) {
    if (state_ != MissionState::TEST) return false;
    transitionTo(MissionState::PRE_LAUNCH, now_ms);
    return true;
}

bool StateMachine::checkAbort(const StateMachineInputs& in, uint32_t now_ms) {
    // Endzustände nicht erneut abbrechen (sonst würde z.B. der Landestoß auf
    // die Wegezelle die abgeschlossene Mission zurück nach ABORT werfen).
    if (state_ == MissionState::ABORT || state_ == MissionState::SAFE) return false;

    if (in.operator_abort) {
        transitionTo(MissionState::ABORT, now_ms);
        return true;
    }

    // Force-Limit nur im Experiment: nur dort hat der Arm Kontakt zum Target.
    if (state_ == MissionState::EXPERIMENT && in.force_load_n > MissionConfig::F_MAX_N) {
        transitionTo(MissionState::ABORT, now_ms);
        return true;
    }
    return false;
}

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
    if (in.soe) {
        t_soe_ms_ = now_ms;
        transitionTo(MissionState::EXPERIMENT, now_ms);
    }
}

void StateMachine::handleExperiment(uint32_t now_ms) {
    // TODO: Hier gehört die eigentliche Experimentsequenz hin - HDRM öffnen,
    // Arm ausfahren, Docking Target 1 + 2, Arm einfahren, HDRM schließen,
    // Netz auslösen. Erfordert Arm-Ansteuerung und Endschalter, die es im
    // Projekt noch nicht gibt. Bis dahin nur der harte Zeit-Cutoff.
    if (now_ms - t_soe_ms_ >= MissionConfig::T_EXPERIMENT_MS) {
        transitionTo(MissionState::SAFE, now_ms);
    }
}

void StateMachine::handleTest(const StateMachineInputs& in, uint32_t now_ms) {
    // Der Flug schlägt den Bodentest: kommt SODS, während noch getestet wird,
    // geht es sofort in die reguläre Sequenz (System stoppt dabei die Aktoren).
    if (in.sods) {
        transitionTo(MissionState::ARMED, now_ms);
    }
}
