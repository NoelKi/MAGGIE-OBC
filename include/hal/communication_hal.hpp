#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file communication_hal.hpp
 * @brief Hardware Abstraction Layer for Communication Interfaces
 * 
 * Provides unified interface for up/down-link signals
 */

class CommunicationHAL {
public:
    /**
     * @brief Constructor for Communication HAL
     * @param pin_uplink_plus Up-link positive signal
     * @param pin_uplink_minus Up-link negative signal
     */
    CommunicationHAL(uint8_t pin_uplink_plus, uint8_t pin_uplink_minus);
    
    /**
     * @brief Initialize communication pins
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Read uplink signal
     * @return Signal level (0 or 1)
     */
    uint8_t readUplinkPlus();
    
    /**
     * @brief Read uplink negative signal
     * @return Signal level (0 or 1)
     */
    uint8_t readUplinkMinus();
    
    /**
     * @brief Check if uplink is connected
     * @return true if uplink signal detected
     */
    bool isUplinkActive();
    
private:
    uint8_t pin_uplink_plus_;
    uint8_t pin_uplink_minus_;
    bool initialized_ = false;
};
