#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file uart_hal.hpp
 * @brief Hardware Abstraction Layer for UART Communication
 * 
 * Provides unified interface for serial communication via UART
 */

class UARTHAL {
public:
    /**
     * @brief Constructor for UART HAL
     * @param tx_pin Transmit pin
     * @param rx_pin Receive pin
     * @param uart_port Hardware UART port (1, 2, 3, etc.)
     */
    UARTHAL(uint8_t tx_pin, uint8_t rx_pin, uint8_t uart_port = 1);
    
    /**
     * @brief Initialize UART
     * @param baudrate Serial communication speed
     * @return true if successful
     */
    bool init(uint32_t baudrate = 115200);
    
    /**
     * @brief Send data
     * @param data Pointer to data buffer
     * @param length Number of bytes to send
     * @return Number of bytes sent
     */
    size_t send(const uint8_t* data, size_t length);
    
    /**
     * @brief Receive data
     * @param data Pointer to buffer for received data
     * @param max_length Maximum number of bytes to read
     * @return Number of bytes received
     */
    size_t receive(uint8_t* data, size_t max_length);
    
    /**
     * @brief Check if data is available
     * @return true if data is ready to read
     */
    bool available() const;
    
    /**
     * @brief Flush transmit buffer
     */
    void flush();
    
private:
    uint8_t tx_pin_;
    uint8_t rx_pin_;
    uint8_t uart_port_;
    uint32_t baudrate_ = 115200;
    bool initialized_ = false;
};

#endif /* UART_HAL_HPP */
