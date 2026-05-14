#include "hal/led_hal.hpp"

LEDHAL::LEDHAL(uint8_t pin, uint8_t led_id)
    : pin_(pin), led_id_(led_id) {
}

bool LEDHAL::init() {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);  // Start with LED off
    
    initialized_ = true;
    return true;
}

void LEDHAL::setBrightness(uint8_t brightness) {
    if (!initialized_) return;
    
    current_brightness_ = brightness;
    analogWrite(pin_, brightness);
}

void LEDHAL::toggle() {
    setBrightness(current_brightness_ > 0 ? 0 : 255);
}

void LEDHAL::blink(uint16_t on_ms, uint16_t off_ms, uint8_t count) {
    for (uint8_t i = 0; i < count; ++i) {
        setBrightness(255);
        delay(on_ms);
        
        setBrightness(0);
        delay(off_ms);
    }
}
