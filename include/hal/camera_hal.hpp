#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file camera_hal.hpp
 * @brief Hardware Abstraction Layer for Camera Communication
 * 
 * Provides UART interface for camera modules
 */

class CameraHAL {
public:
    /**
     * @brief Constructor for Camera HAL
     * @param tx_pin Transmit pin
     * @param rx_pin Receive pin
     * @param uart_port Hardware UART port
     * @param camera_id Camera identifier (1-3)
     */
    CameraHAL(uint8_t tx_pin, uint8_t rx_pin, uint8_t uart_port, uint8_t camera_id = 0);
    
    /**
     * @brief Initialize camera UART
     * @param baudrate Serial communication speed
     * @return true if successful
     */
    bool init(uint32_t baudrate = 115200);
    
    /**
     * @brief Send command to camera
     * @param command Command data
     * @param length Length of command
     * @return Number of bytes sent
     */
    size_t sendCommand(const uint8_t* command, size_t length);
    
    /**
     * @brief Receive data from camera
     * @param buffer Buffer for received data
     * @param max_length Maximum bytes to read
     * @return Number of bytes received
     */
    size_t receiveData(uint8_t* buffer, size_t max_length);
    
    /**
     * @brief Check if data is available from camera
     * @return true if data is ready
     */
    bool available();
    
    /**
     * @brief Capture image (simplified interface)
     * @return true if capture was successful
     */
    bool captureImage();
    
    /**
     * @brief Get camera ID
     * @return Camera identifier
     */
    uint8_t getCameraID() const { return camera_id_; }
    
private:
    uint8_t tx_pin_;
    uint8_t rx_pin_;
    uint8_t uart_port_;
    uint8_t camera_id_;
    bool initialized_ = false;
};
