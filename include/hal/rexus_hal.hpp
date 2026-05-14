#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file rexus_hal.hpp
 * @brief Hardware Abstraction Layer for REXUS Signal Interface
 * 
 * Provides interface for REXUS experiment signals:
 * - L0_t: Launch signal
 * - SOE_i: Start of Experiment
 * - SODS_i: Start of Descent
 */

class REXUSHAL {
public:
    /**
     * @brief Constructor for REXUS HAL
     * @param pin_l0_t Launch signal pin
     * @param pin_soe_i Start of Experiment pin
     * @param pin_sods_i Start of Descent pin
     */
    REXUSHAL(uint8_t pin_l0_t, uint8_t pin_soe_i, uint8_t pin_sods_i);
    
    /**
     * @brief Initialize REXUS signal pins
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Read L0_t signal
     * @return true if signal is active
     */
    bool readL0T();
    
    /**
     * @brief Read SOE_i signal
     * @return true if signal is active
     */
    bool readSOE();
    
    /**
     * @brief Read SODS_i signal
     * @return true if signal is active
     */
    bool readSODS();
    
    /**
     * @brief Get all signal states
     * @param l0_t Reference to L0_t state
     * @param soe Reference to SOE state
     * @param sods Reference to SODS state
     */
    void getAllSignals(bool& l0_t, bool& soe, bool& sods);
    
    /**
     * @brief Check if experiment has started (L0_t signal)
     * @return true if launched
     */
    bool isLaunched();
    
    /**
     * @brief Check if experiment data collection has started
     * @return true if experiment active
     */
    bool isExperimentActive();
    
    /**
     * @brief Check if spacecraft is descending
     * @return true if descending
     */
    bool isDescending();
    
private:
    uint8_t pin_l0_t_;
    uint8_t pin_soe_i_;
    uint8_t pin_sods_i_;
    bool initialized_ = false;
};
