#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file force_sensor_hal.hpp
 * @brief Hardware Abstraction Layer for analog Force Sensors (Sensor 2)
 * 
 * Provides unified interface for reading analog force measurements
 * from X, Y, Z axes of the custom force sensor (Sensor 2)
 */

struct ForceSensorReading {
    float force_x;          ///< Force in X direction (Newtons or custom units)
    float force_y;          ///< Force in Y direction
    float force_z;          ///< Force in Z direction
    uint32_t raw_x;         ///< Raw ADC value X
    uint32_t raw_y;         ///< Raw ADC value Y
    uint32_t raw_z;         ///< Raw ADC value Z
    uint32_t timestamp;     ///< Timestamp in milliseconds
    bool valid;             ///< Whether reading is valid
};

class ForceSensorHAL {
public:
    /**
     * @brief Constructor for Force Sensor HAL
     * @param pin_x Analog pin for X-axis
     * @param pin_y Analog pin for Y-axis
     * @param pin_z Analog pin for Z-axis
     */
    ForceSensorHAL(uint8_t pin_x, uint8_t pin_y, uint8_t pin_z);
    
    /**
     * @brief Initialize the force sensor
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Read force measurements from all three axes
     * @param reading Reference to ForceSensorReading struct to fill
     * @param calibration_x Calibration factor for X axis (raw -> force)
     * @param calibration_y Calibration factor for Y axis
     * @param calibration_z Calibration factor for Z axis
     * @return true if reading was successful
     */
    bool read(ForceSensorReading& reading, 
              float calibration_x = 1.0f,
              float calibration_y = 1.0f,
              float calibration_z = 1.0f);
    
    /**
     * @brief Set calibration offset (zero point)
     * @param offset_x Zero offset for X axis
     * @param offset_y Zero offset for Y axis
     * @param offset_z Zero offset for Z axis
     */
    void setOffset(uint32_t offset_x, uint32_t offset_y, uint32_t offset_z);
    
    /**
     * @brief Get current offset values
     */
    void getOffset(uint32_t& offset_x, uint32_t& offset_y, uint32_t& offset_z) const;
    
private:
    uint8_t pin_x_;
    uint8_t pin_y_;
    uint8_t pin_z_;
    
    uint32_t offset_x_ = 0;
    uint32_t offset_y_ = 0;
    uint32_t offset_z_ = 0;
    
    bool initialized_ = false;
};
