#include "hal/motor_hal.hpp"

MotorHAL::MotorHAL(uint8_t pin_a, uint8_t pin_b, uint8_t motor_id)
    : pin_a_(pin_a), pin_b_(pin_b), motor_id_(motor_id) {
}

bool MotorHAL::init() {
    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);
    
    // Set both channels to 0 (stop)
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
    // Teensy 4.1 PWM frequency control
    // This is a simplified version; actual implementation may vary
    // based on the pin and timer used
    // For now, this is a placeholder
    (void)frequency;
}
