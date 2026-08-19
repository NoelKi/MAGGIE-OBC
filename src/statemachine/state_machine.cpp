#include "statemachine/state_machine.hpp"
#include <Arduino.h>

const char* StateMachine::toString(MissionState s) {
    switch (s) {
        case MissionState::PRE_LAUNCH: return "PRE_LAUNCH";
        case MissionState::ABORT:      return "ABORT";
        case MissionState::TEST:       return "TEST";
    }
    return "UNKNOWN";
}

void StateMachine::init() {
    state_ = MissionState::PRE_LAUNCH;
}

void StateMachine::transitionTo(MissionState next, uint32_t now_ms) {
    if (next == state_) return;
    Serial.printf("INFO  [StateMachine]: %s -> %s (t=%lu ms)\n",
                  toString(state_), toString(next),
                  static_cast<unsigned long>(now_ms));
    state_ = next;
}

void StateMachine::update(uint32_t now_ms, const StateMachineInputs& in) {
    // ABORT ist ein Endzustand: einmal drin, hilft nur noch ein Reset.
    if (state_ == MissionState::ABORT) return;

    if (in.operator_abort) {
        transitionTo(MissionState::ABORT, now_ms);
    }

    // PRE_LAUNCH und TEST haben keine zeit- oder signalgesteuerten Uebergaenge -
    // gewechselt wird nur per Telecommand (enterTest/exitTest).
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
