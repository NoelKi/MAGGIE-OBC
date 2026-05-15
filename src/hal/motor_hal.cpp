#include "hal/motor_hal.hpp"

MotorHAL::MotorHAL(uint8_t pin_a, uint8_t pin_b, uint8_t motor_id)
    : pin_a_(pin_a), pin_b_(pin_b), motor_id_(motor_id) {
}

bool MotorHAL::init() {
    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);

    setPWMFrequency(20000); // above audible range, within DRV8871 spec

    analogWrite(pin_a_, 0);
    analogWrite(pin_b_, 0);

    initialized_ = true;
    return true;
}

void MotorHAL::setSpeed(int16_t speed) {
    if (!initialized_) return;
    
    // Clamp speed to valid range
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    
    current_speed_ = speed;
    
    if (speed > 0) {
        // Forward: Channel A active, Channel B off
        analogWrite(pin_a_, speed);
        analogWrite(pin_b_, 0);
    } else if (speed < 0) {
        // Reverse: Channel B active, Channel A off
        analogWrite(pin_a_, 0);
        analogWrite(pin_b_, -speed);
    } else {
        // Stop: Both channels off
        analogWrite(pin_a_, 0);
        analogWrite(pin_b_, 0);
    }
}

void MotorHAL::stop() {
    setSpeed(0);
}

void MotorHAL::setPWMFrequency(uint32_t frequency) {
    analogWriteFrequency(pin_a_, frequency);
    analogWriteFrequency(pin_b_, frequency);
}
