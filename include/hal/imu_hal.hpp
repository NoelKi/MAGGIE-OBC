#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file imu_hal.hpp
 * @brief Hardware Abstraction Layer for IMU (Gyroscope & Accelerometer)
 * 
 * Provides unified interface for reading acceleration and rotation data
 */

struct IMUReading {
    float accel_x;          ///< Acceleration X (m/s²)
    float accel_y;          ///< Acceleration Y (m/s²)
    float accel_z;          ///< Acceleration Z (m/s²)
    float gyro_x;           ///< Angular velocity X (°/s)
    float gyro_y;           ///< Angular velocity Y (°/s)
    float gyro_z;           ///< Angular velocity Z (°/s)
    uint32_t timestamp;     ///< Timestamp in milliseconds
    bool valid;             ///< Whether reading is valid
};

class IMUHAL {
public:
    /**
     * @brief Constructor for IMU HAL
     * @param cs_accel Chip Select for Accelerometer
     * @param cs_gyro Chip Select for Gyroscope
     * @param spi_port SPI port number
     */
    IMUHAL(uint8_t cs_accel, uint8_t cs_gyro, uint8_t spi_port = 1);
    
    /**
     * @brief Initialize IMU (SPI communication)
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Read IMU data
     * @param reading Reference to IMUReading struct to fill
     * @return true if reading was successful
     */
    bool read(IMUReading& reading);
    
    /**
     * @brief Calibrate IMU (zero offset)
     * @param samples Number of samples for calibration
     */
    void calibrate(uint16_t samples = 100);
    
    /**
     * @brief Self-test IMU
     * @return true if all tests pass
     */
    bool selfTest();
    
private:
    uint8_t cs_accel_;
    uint8_t cs_gyro_;
    uint8_t spi_port_;
    bool initialized_ = false;
};

#endif /* IMU_HAL_HPP */
