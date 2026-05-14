#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file led_hal.hpp
 * @brief Hardware Abstraction Layer for LED Control (PWM)
 * 
 * Provides PWM brightness control for status LEDs
 */

class LEDHAL {
public:
    /**
     * @brief Constructor for LED HAL
     * @param pin PWM pin for LED
     * @param led_id LED identifier
     */
    LEDHAL(uint8_t pin, uint8_t led_id = 0);
    
    /**
     * @brief Initialize LED pin
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Set LED brightness
     * @param brightness 0-255 (0 = off, 255 = full brightness)
     */
    void setBrightness(uint8_t brightness);
    
    /**
     * @brief Turn LED on (full brightness)
     */
    void on() { setBrightness(255); }
    
    /**
     * @brief Turn LED off
     */
    void off() { setBrightness(0); }
    
    /**
     * @brief Toggle LED
     */
    void toggle();
    
    /**
     * @brief Get current brightness
     * @return Current brightness value (0-255)
     */
    uint8_t getBrightness() const { return current_brightness_; }
    
    /**
     * @brief Blink LED (blocking)
     * @param on_ms Time LED is on
     * @param off_ms Time LED is off
     * @param count Number of blinks
     */
    void blink(uint16_t on_ms, uint16_t off_ms, uint8_t count);
    
private:
    uint8_t pin_;
    uint8_t led_id_;
    uint8_t current_brightness_ = 0;
    bool initialized_ = false;
};

#endif /* LED_HAL_HPP */
