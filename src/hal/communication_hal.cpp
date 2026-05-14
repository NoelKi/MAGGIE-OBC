#include "hal/communication_hal.hpp"

CommunicationHAL::CommunicationHAL(uint8_t pin_uplink_plus, uint8_t pin_uplink_minus)
    : pin_uplink_plus_(pin_uplink_plus), pin_uplink_minus_(pin_uplink_minus) {
}

bool CommunicationHAL::init() {
    // Initialize pins as digital inputs
    pinMode(pin_uplink_plus_, INPUT);
    pinMode(pin_uplink_minus_, INPUT);
    
    initialized_ = true;
    return true;
}

uint8_t CommunicationHAL::readUplinkPlus() {
    if (!initialized_) return 0;
    return digitalRead(pin_uplink_plus_);
}

uint8_t CommunicationHAL::readUplinkMinus() {
    if (!initialized_) return 0;
    return digitalRead(pin_uplink_minus_);
}

bool CommunicationHAL::isUplinkActive() {
    if (!initialized_) return false;
    // Uplink is active if either signal is high
    return (digitalRead(pin_uplink_plus_) == HIGH) || 
           (digitalRead(pin_uplink_minus_) == HIGH);
}
