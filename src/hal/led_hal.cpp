#include "hal/led_hal.hpp"

LEDHAL::LEDHAL(uint8_t pin_r, uint8_t pin_g, uint8_t pin_b, uint8_t led_id)
    : pin_r_(pin_r), pin_g_(pin_g), pin_b_(pin_b), led_id_(led_id) {
}

bool LEDHAL::init() {
    pinMode(pin_r_, OUTPUT);
    pinMode(pin_g_, OUTPUT);
    pinMode(pin_b_, OUTPUT);
    applyRaw(0, 0, 0);
    initialized_ = true;
    return true;
}

void LEDHAL::applyRaw(uint8_t r, uint8_t g, uint8_t b) {
    analogWrite(pin_r_, r);
    analogWrite(pin_g_, g);
    analogWrite(pin_b_, b);
}

void LEDHAL::setColor(uint8_t r, uint8_t g, uint8_t b) {
    if (!initialized_) return;
    r_ = r;
    g_ = g;
    b_ = b;
    brightness_ = 255;
    applyRaw(r_, g_, b_);
}

void LEDHAL::setBrightness(uint8_t brightness) {
    if (!initialized_) return;
    brightness_ = brightness;
    applyRaw(
        static_cast<uint8_t>((r_ * brightness) / 255),
        static_cast<uint8_t>((g_ * brightness) / 255),
        static_cast<uint8_t>((b_ * brightness) / 255)
    );
}

void LEDHAL::toggle() {
    if (brightness_ > 0) {
        applyRaw(0, 0, 0);
        brightness_ = 0;
    } else {
        setBrightness(255);
    }
}

void LEDHAL::blink(uint16_t on_ms, uint16_t off_ms, uint8_t count) {
    for (uint8_t i = 0; i < count; ++i) {
        setBrightness(255);
        delay(on_ms);
        applyRaw(0, 0, 0);
        delay(off_ms);
    }
    brightness_ = 0;
}
