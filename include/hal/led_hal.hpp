#pragma once

#include <cstdint>
#include <Arduino.h>

class LEDHAL {
public:
    LEDHAL(uint8_t pin_r, uint8_t pin_g, uint8_t pin_b, uint8_t led_id = 0);

    bool init();

    void setColor(uint8_t r, uint8_t g, uint8_t b);

    // Scales the current color uniformly (0 = off, 255 = full)
    void setBrightness(uint8_t brightness);

    void on()  { setColor(r_, g_, b_); }
    void off() { applyRaw(0, 0, 0); }
    void toggle();

    void blink(uint16_t on_ms, uint16_t off_ms, uint8_t count);

    uint8_t getBrightness() const { return brightness_; }

private:
    uint8_t pin_r_, pin_g_, pin_b_;
    uint8_t led_id_;
    uint8_t r_ = 255, g_ = 255, b_ = 255; // color to restore on on()/setBrightness()
    uint8_t brightness_ = 0;
    bool initialized_ = false;

    void applyRaw(uint8_t r, uint8_t g, uint8_t b);
};
