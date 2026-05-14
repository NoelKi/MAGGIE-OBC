#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file motor_hal.hpp
 * @brief Hardware Abstraction Layer for Motor Control (DRV8871)
 * 
 * Provides PWM control interface for three motors with A/B channels
 */

class MotorHAL {
public:
    /**
     * @brief Constructor for Motor HAL
     * @param pin_a Channel A (PWM)
     * @param pin_b Channel B (PWM)
     * @param motor_id Motor identifier (1-3)
     */
    MotorHAL(uint8_t pin_a, uint8_t pin_b, uint8_t motor_id = 0);
    
    /**
     * @brief Initialize motor pins
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Set motor speed and direction
     * @param speed -255 (full reverse) to +255 (full forward), 0 = stop
     */
    void setSpeed(int16_t speed);
    
    /**
     * @brief Stop motor immediately
     */
    void stop();
    
    /**
     * @brief Get current motor speed
     * @return Current speed value
     */
    int16_t getSpeed() const { return current_speed_; }
    
    /**
     * @brief Set PWM frequency
     * @param frequency Frequency in Hz
     */
    void setPWMFrequency(uint32_t frequency);
    
private:
    uint8_t pin_a_;
    uint8_t pin_b_;
    uint8_t motor_id_;
    int16_t current_speed_ = 0;
    bool initialized_ = false;
};
