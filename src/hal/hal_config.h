#pragma once

/**
 * HAL Configuration - Zentrale Pin-Definitionen für Teensy 4.1 MAGGIE-OBC
 * 
 * Alle Hardware-Pins sind hier zentral definiert.
 * Bei Hardware-Änderungen: Nur diese Datei anpassen!
 */

// ============================================================================
// I2C PINS (Wire / I2C0)
// ============================================================================
#define HAL_I2C_SDA 18      // Pin 18 (SDA)
#define HAL_I2C_SCL 19      // Pin 19 (SCL)
#define HAL_I2C_FREQ 400000 // 400 kHz Standard

// ============================================================================
// SPI PINS (SPI0)
// ============================================================================
#define HAL_SPI_MOSI 11     // Pin 11 (MOSI)
#define HAL_SPI_MISO 12     // Pin 12 (MISO)
#define HAL_SPI_SCK  13     // Pin 13 (SCK)
#define HAL_SPI_CS_PRESSURE 10  // Pin 10 (CS für Drucksensor MS5607)
#define HAL_SPI_CS_SD 4          // Pin 4 (CS für SD-Card)
#define HAL_SPI_FREQ 10000000    // 10 MHz

// ============================================================================
// CAN BUS PINS (FlexCAN0)
// ============================================================================
#define HAL_CAN_TX 22       // Pin 22 (CAN TX)
#define HAL_CAN_RX 23       // Pin 23 (CAN RX)
#define HAL_CAN_BAUD 500000 // 500 kbps (Avionik Standard)

// ============================================================================
// PWM PINS (für HDRM Motor Control)
// ============================================================================
#define HAL_PWM_SPEED 9     // Pin 9 (PWM Speed 0-100%)
#define HAL_PWM_DIR1 7      // Pin 7 (Direction 1)
#define HAL_PWM_DIR2 8      // Pin 8 (Direction 2)
#define HAL_PWM_ENABLE 6    // Pin 6 (Enable/Disable)
#define HAL_PWM_FREQ 20000  // 20 kHz PWM Frequency

// ============================================================================
// ADC PINS (für Weight Sensors / Load Cells)
// ============================================================================
#define HAL_ADC_SCALE1 A0   // Pin A0 (Gewichtssensor 1)
#define HAL_ADC_SCALE2 A1   // Pin A1 (Gewichtssensor 2)
#define HAL_ADC_RESOLUTION 12 // 12-bit ADC

// ============================================================================
// UART PINS (für Telemetrie Serial)
// ============================================================================
#define HAL_UART_TX 1       // Pin 1 (TX via USB)
#define HAL_UART_RX 0       // Pin 0 (RX via USB)
#define HAL_UART_BAUD 115200 // 115200 baud

// ============================================================================
// LED & DEBUG PINS
// ============================================================================
#define HAL_LED_PIN 13      // Pin 13 (onboard LED)
#define HAL_STATUS_LED 3    // Pin 3 (Status LED)

// ============================================================================
// TIMEOUTS & DEFAULTS
// ============================================================================
#define HAL_SPI_TIMEOUT_MS 1000
#define HAL_I2C_TIMEOUT_MS 500
#define HAL_CAN_TIMEOUT_MS 100
