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

void StateMachine::init(MissionState start) {
    state_ = start;
}

MissionState StateMachine::stateAfterReboot(uint8_t stored, bool valid) {
    if (!valid) return MissionState::PRE_LAUNCH;

    switch (static_cast<MissionState>(stored)) {
        // Ein Abbruch bleibt ein Abbruch - ein Reset darf ihn nicht aufheben.
        case MissionState::ABORT:      return MissionState::ABORT;

        // TEST ist ein reiner Bodenmodus: Nach einem Neustart sind die Aktoren
        // wieder gesperrt, bis erneut TEST_ENTER kommt.
        case MissionState::TEST:       return MissionState::PRE_LAUNCH;

        case MissionState::PRE_LAUNCH: return MissionState::PRE_LAUNCH;
    }
    // Unbekannter Wert (z.B. aus einem anderen Firmware-Stand) -> sicherer Grundzustand.
    return MissionState::PRE_LAUNCH;
}

void StateMachine::transitionTo(MissionState next, uint32_t now_ms) {
    if (next == state_) return;
    Serial.printf("INFO  [StateMachine]: %s -> %s (t=%lu ms)\n",
                  toString(state_), toString(next),
                  static_cast<unsigned long>(now_ms));
    state_ = next;
}

void StateMachine::update(uint32_t now_ms, const StateMachineInputs& in) {
    // ABORT verlaesst nur der Telecommand STATE_RESET (resetFromAbort).
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

bool StateMachine::resetFromAbort(uint32_t now_ms) {
    if (state_ != MissionState::ABORT) {
        Serial.printf("WARN  [StateMachine]: STATE_RESET abgelehnt - Zustand ist %s.\n",
                      toString(state_));
        return false;
    }
    transitionTo(MissionState::PRE_LAUNCH, now_ms);
    return true;
}
