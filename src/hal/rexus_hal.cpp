#include "hal/rexus_hal.hpp"

REXUSHAL::REXUSHAL(uint8_t pin_l0_t, uint8_t pin_soe_i, uint8_t pin_sods_i)
    : pin_l0_t_(pin_l0_t), pin_soe_i_(pin_soe_i), pin_sods_i_(pin_sods_i) {
}

bool REXUSHAL::init() {
    // Initialize pins as digital inputs
    pinMode(pin_l0_t_, INPUT);
    pinMode(pin_soe_i_, INPUT);
    pinMode(pin_sods_i_, INPUT);
    
    initialized_ = true;
    return true;
}

bool REXUSHAL::readL0T() {
    if (!initialized_) return false;
    return digitalRead(pin_l0_t_) == HIGH;
}

bool REXUSHAL::readSOE() {
    if (!initialized_) return false;
    return digitalRead(pin_soe_i_) == HIGH;
}

bool REXUSHAL::readSODS() {
    if (!initialized_) return false;
    return digitalRead(pin_sods_i_) == HIGH;
}

void REXUSHAL::getAllSignals(bool& l0_t, bool& soe, bool& sods) {
    if (!initialized_) {
        l0_t = false;
        soe = false;
        sods = false;
        return;
    }
    
    l0_t = readL0T();
    soe = readSOE();
    sods = readSODS();
}

bool REXUSHAL::isLaunched() {
    return readL0T();
}

bool REXUSHAL::isExperimentActive() {
    return readSOE();
}

bool REXUSHAL::isDescending() {
    return readSODS();
}
