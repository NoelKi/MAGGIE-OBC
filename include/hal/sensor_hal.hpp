#pragma once

#include <cstdint>
#include "pin_config.hpp"

// Include all HAL modules
#include "hal/force_sensor_hal.hpp"
#include "hal/motor_hal.hpp"
#include "hal/led_hal.hpp"
#include "hal/uart_hal.hpp"
#include "hal/imu_hal.hpp"
#include "hal/rexus_hal.hpp"
#include "hal/communication_hal.hpp"
#include "hal/camera_hal.hpp"

/**
 * @file sensor_hal.hpp
 * @brief Central HAL configuration for all sensors and subsystems
 * 
 * Maps pin_config.hpp definitions to actual hardware interfaces
 * and provides centralized access to all HAL modules
 */

// ===========================================================================
// HX711 Weight Sensor (Sensor 1 - Purchased Scale)
// ===========================================================================
constexpr uint8_t HX711_DOUT_PIN = PIN_HX711_DOUT;  // Pin 2
constexpr uint8_t HX711_SCK_PIN = PIN_HX711_SCK;    // Pin 3

// ===========================================================================
// Force Sensor 2 (Custom Analog)
// ===========================================================================
constexpr uint8_t FORCE_SENSOR_2_X_PIN = PIN_FORCE_X_2;  // Pin 30
constexpr uint8_t FORCE_SENSOR_2_Y_PIN = PIN_FORCE_Y_2;  // Pin 31
constexpr uint8_t FORCE_SENSOR_2_Z_PIN = PIN_FORCE_Z_2;  // Pin 32
constexpr uint8_t FORCE_SENSOR_2_CLK_PIN = PIN_FORCE_CLK_2;      // Pin 9
constexpr uint8_t FORCE_SENSOR_2_CS_PIN = PIN_CHIP_SELECT_FORCE;  // Pin 10

// ===========================================================================
// SPI Configuration
// ===========================================================================
constexpr uint8_t SPI1_MOSI_PIN = PIN_SPI1_MOSI;  // Pin 11
constexpr uint8_t SPI1_MISO_PIN = PIN_SPI1_MISO;  // Pin 12
constexpr uint8_t SPI1_SCK_PIN = PIN_SCK;         // Pin 13

// ===========================================================================
// IMU/Accelerometer/Gyroscope
// ===========================================================================
constexpr uint8_t CS_ACCEL_PIN = PIN_CS_ACCEL;  // Pin 37
constexpr uint8_t CS_GYRO_PIN = PIN_CS_GYRO;    // Pin 36

// ===========================================================================
// UART Communication
// ===========================================================================
constexpr uint8_t ARM_TX_PIN = PIN_ARM_TX;  // Pin 0
constexpr uint8_t ARM_RX_PIN = PIN_ARM_RX;  // Pin 1

// ===========================================================================
// Camera Communication
// ===========================================================================
constexpr uint8_t CAM1_TX_PIN = PIN_CAM1_TX;           // Pin 7
constexpr uint8_t CAM_MAIN_RX_PIN = PIN_CAM_MAIN_RX;   // Pin 8
constexpr uint8_t CAM_BACKUP_RX_PIN = PIN_CAM_BACKUP_RX;  // Pin 24
constexpr uint8_t CAM3_TX_PIN = PIN_CAM3_TX;           // Pin 25

// ===========================================================================
// Motor Control
// ===========================================================================
constexpr uint8_t MOTOR_1_A_PIN = PIN_M1_A;  // Pin 15
constexpr uint8_t MOTOR_1_B_PIN = PIN_M1_B;  // Pin 14
constexpr uint8_t MOTOR_2_A_PIN = PIN_M2_A;  // Pin 23
constexpr uint8_t MOTOR_2_B_PIN = PIN_M2_B;  // Pin 22
constexpr uint8_t MOTOR_3_A_PIN = PIN_M3_A;  // Pin 18
constexpr uint8_t MOTOR_3_B_PIN = PIN_M3_B;  // Pin 19

// ===========================================================================
// LED Control
// ===========================================================================
constexpr uint8_t LED_PWM_1_PIN = PIN_LED_PWM_1;  // Pin 5
constexpr uint8_t LED_PWM_2_PIN = PIN_LED_PWM_2;  // Pin 6

// ===========================================================================
// REXUS Signals
// ===========================================================================
constexpr uint8_t REXUS_L0_T_PIN = PIN_L0_T;       // Pin 40
constexpr uint8_t REXUS_SOE_I_PIN = PIN_SOE_I;     // Pin 39
constexpr uint8_t REXUS_SODS_I_PIN = PIN_SODS_I;   // Pin 38

// ===========================================================================
// Up/Down-link
// ===========================================================================
constexpr uint8_t UPDOWNLINK_PLUS_PIN = PIN_UPDOWNLINK_PLUS;   // Pin 34
constexpr uint8_t UPDOWNLINK_MINUS_PIN = PIN_UPDOWNLINK_MINUS; // Pin 35

// ===========================================================================
// Temperature Sensor
// ===========================================================================
constexpr uint8_t CS_TEMP_PIN = PIN_CS_TEMP;  // Pin 26
