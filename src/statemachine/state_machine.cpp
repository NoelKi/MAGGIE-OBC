#include "statemachine/state_machine.hpp"
#include "mission_timeline.hpp"
#include <Arduino.h>

const char* StateMachine::toString(MissionState s) {
    switch (s) {
        case MissionState::PRE_LAUNCH: return "PRE_LAUNCH";
        case MissionState::ARMED:      return "ARMED";
        case MissionState::ASCENT:     return "ASCENT";
        case MissionState::EXPERIMENT: return "EXPERIMENT";
        case MissionState::ROBOT_ARM:  return "ROBOT_ARM";
        case MissionState::DESCENT:    return "DESCENT";
        case MissionState::TEST:       return "TEST";
    }
    return "UNKNOWN";
}

void StateMachine::init(MissionState start, uint32_t now_ms, uint32_t elapsed_ms) {
    state_   = start;
    base_ms_ = now_ms - elapsed_ms;
}

MissionState StateMachine::stateAfterReboot(uint8_t stored, bool valid) {
    if (!valid) return MissionState::PRE_LAUNCH;

    switch (static_cast<MissionState>(stored)) {
        // Flug: genau dort weitermachen, wo der Neustart unterbrochen hat.
        case MissionState::ARMED:
        case MissionState::ASCENT:
        case MissionState::EXPERIMENT:
        case MissionState::ROBOT_ARM:
        case MissionState::DESCENT:
            return static_cast<MissionState>(stored);

        // TEST ist ein reiner Bodenmodus: Nach einem Neustart sind die Aktoren
        // wieder gesperrt, bis erneut TEST_ENTER kommt.
        case MissionState::TEST:       return MissionState::PRE_LAUNCH;

        case MissionState::PRE_LAUNCH: return MissionState::PRE_LAUNCH;
    }
    // Unbekannter Wert (z.B. 5 = altes ABORT oder ein anderer Firmware-Stand)
    // -> sicherer Grundzustand.
    return MissionState::PRE_LAUNCH;
}

void StateMachine::transitionTo(MissionState next, uint32_t now_ms) {
    if (next == state_) return;
    Serial.printf("INFO  [StateMachine]: %s -> %s (t=%lu ms)\n",
                  toString(state_), toString(next),
                  static_cast<unsigned long>(now_ms));

    // Neuer Zeitbezug bei LO (ASCENT) und bei t0 (EXPERIMENT). ROBOT_ARM und
    // DESCENT behalten t0 - ihre Zeiten in mission_timeline.hpp zaehlen ab dort.
    if (next == MissionState::ASCENT || next == MissionState::EXPERIMENT) {
        base_ms_ = now_ms;
    }
    state_ = next;
}

void StateMachine::update(uint32_t now_ms, const StateMachineInputs& in) {
    const uint32_t t = elapsed(now_ms);

    // Vor dem Flug: das SPAETESTE anliegende Signal entscheidet (siehe .hpp).
    if (isGround(state_) || state_ == MissionState::ARMED) {
        if (in.soe) {
            transitionTo(MissionState::EXPERIMENT, now_ms);
        } else if (in.lo) {
            transitionTo(MissionState::ASCENT, now_ms);
        } else if (in.sods) {
            transitionTo(MissionState::ARMED, now_ms);   // auch aus TEST
        }
        return;
    }

    switch (state_) {
        case MissionState::ASCENT:
            if (in.soe || t >= timeline::ASCENT_MAX_MS) {
                transitionTo(MissionState::EXPERIMENT, now_ms);
            }
            break;

        case MissionState::EXPERIMENT:
            if (t >= timeline::DESCENT_AT_MS) {
                transitionTo(MissionState::DESCENT, now_ms);
            } else if (in.hdrms_open_done) {
                transitionTo(MissionState::ROBOT_ARM, now_ms);
            }
            break;

        case MissionState::ROBOT_ARM:
            if (t >= timeline::DESCENT_AT_MS) transitionTo(MissionState::DESCENT, now_ms);
            break;

        case MissionState::DESCENT:
            break;   // nur per Telecommand STATE_RESET

        default:
            break;   // Boden und ARMED: oben behandelt
    }
}

bool StateMachine::enterTest(uint32_t now_ms) {
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

bool StateMachine::resetToPreLaunch(uint32_t now_ms) {
    // Aus JEDEM Zustand - Testwerkzeug, im Flug gibt es keinen Uplink.
    if (state_ == MissionState::PRE_LAUNCH) return false;
    transitionTo(MissionState::PRE_LAUNCH, now_ms);
    return true;
}
