#pragma once

#include <cstdint>
using std::uint8_t;
using std::uint32_t;

/**
 * @file pin_config.hpp
 * @brief Central pin assignment for Teensy 4.1 OBC
 *
 * Define all hardware pins in one place - so only this file needs to be
 * changed during rewiring.
 * 
 * Source: Teensy 4.1 Pinout Diagram (MAGGIE OBC v1.6)
 */

// ===========================================================================
// UART / Serial Communication
// ===========================================================================
// ARM Communication (UART1)
static constexpr uint8_t PIN_ARM_TX = 0;       ///< UART1 TX
static constexpr uint8_t PIN_ARM_RX = 1;       ///< UART1 RX

// ===========================================================================
// Force Sensors 1
// ===========================================================================
// Sensor 1 (Force X, Y, Z) - Analog Input
static constexpr uint8_t PIN_FORCE_X_1 = 2;   ///< Analog Input
static constexpr uint8_t PIN_FORCE_Y_1 = 3;   ///< Analog Input
static constexpr uint8_t PIN_FORCE_Z_1 = 4;   ///< Analog Input

// ===========================================================================
// LED PWM Outputs
// ===========================================================================
static constexpr uint8_t PIN_LED_PWM_1 = 5;    ///< LED PWM 1
static constexpr uint8_t PIN_LED_PWM_2 = 6;    ///< LED PWM 2

// ===========================================================================
// Cameras
// ===========================================================================
// Camera 1 (UART2)
static constexpr uint8_t PIN_CAM1_TX = 7;      ///< UART2 TX
static constexpr uint8_t PIN_CAM_MAIN_RX = 8; ///< UART2 RX

// ===========================================================================
// Force CLK 2
// ===========================================================================
static constexpr uint8_t PIN_FORCE_CLK_2 = 9;   ///< Analog Input
static constexpr uint8_t PIN_CHIP_SELECT_FORCE = 10;   ///< Chip Select (Digital OUT)

// ===========================================================================
// SPI Bus
// ===========================================================================
// Standard Teensy 4.1 SPI1 Pins
static constexpr uint8_t PIN_SPI1_MOSI = 11;    ///< MOSI
static constexpr uint8_t PIN_SPI1_MISO = 12;    ///< MISO

// ===========================================================================
// Cameras
// ===========================================================================
// Camera 1 (UART2)
static constexpr uint8_t PIN_CAM_BACKUP_RX = 24;     
static constexpr uint8_t PIN_CAM3_TX = 25; 
static constexpr uint8_t PIN_CS_TEMP = 26; 

// ===========================================================================
// Force CLK 1
// ===========================================================================
static constexpr uint8_t PIN_FORCE_CLK_1 = 27;   ///< Analog Input

// ===========================================================================
// Force Sensors 2
// ===========================================================================
// Sensor 2 (Force X, Y, Z) - Analog Input
static constexpr uint8_t PIN_FORCE_X_2 = 30;   ///< Analog Input
static constexpr uint8_t PIN_FORCE_Y_2 = 31;   ///< Analog Input
static constexpr uint8_t PIN_FORCE_Z_2 = 32;   ///< Analog Input

// ===========================================================================
// Motor PWM Outputs (DRV8871 Driver)
// ===========================================================================
// Motor 2 (Channels A+B)
static constexpr uint8_t PIN_M2_B = 22;        ///< Motor 2 Channel B
static constexpr uint8_t PIN_M2_A = 23;        ///< Motor 2 Channel A

// Motor 3 (Channels A+B)
static constexpr uint8_t PIN_M3_A = 18;        ///< Motor 3 Channel A
static constexpr uint8_t PIN_M3_B = 19;        ///< Motor 3 Channel B

// Motor 1 (Channels A+B)
static constexpr uint8_t PIN_M1_B = 14;        ///< Motor 1 Channel B
static constexpr uint8_t PIN_M1_A = 15;        ///< Motor 1 Channel A
// SCK
static constexpr uint8_t PIN_SCK = 13;        ///< SCK

// ===========================================================================
// REXUS Signals
// ===========================================================================
static constexpr uint8_t PIN_L0_T = 40;        ///< L0_t Signal
static constexpr uint8_t PIN_SOE_I = 39;       ///< SOE_i Signal
static constexpr uint8_t PIN_SODS_I = 38;      ///< SODS_i Signal

// ===========================================================================
// Sensors
// ===========================================================================
// Chip Select Pins
static constexpr uint8_t PIN_CS_ACCEL = 37;    ///< Chip Select Accelerometer
static constexpr uint8_t PIN_CS_GYRO = 36;     ///< Chip Select Gyroscope

// ===========================================================================
// Up/Down-link
// ===========================================================================
static constexpr uint8_t PIN_UPDOWNLINK_MINUS = 35;  ///< updownlink-
static constexpr uint8_t PIN_UPDOWNLINK_PLUS = 34;  ///< updownlink+

// ===========================================================================
// HX711 Aliases (Sensor 1 - Purchased Scale)
// ===========================================================================
static constexpr uint8_t PIN_HX711_DOUT = PIN_FORCE_X_1;  ///< HX711 Data Out (mapped to Sensor 1)
static constexpr uint8_t PIN_HX711_SCK = PIN_FORCE_Y_1;   ///< HX711 Serial Clock (mapped to Sensor 1)
