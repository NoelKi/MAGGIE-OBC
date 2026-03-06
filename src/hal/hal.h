#pragma once

/**
 * MAGGIE HAL (Hardware Abstraction Layer)
 * 
 * Zentrale Header - Importiere diese statt einzelner HAL-Module
 * 
 * Submodule:
 * - hal_config.h   : Pin-Definitionen & Konfiguration
 * - hal_gpio.h     : GPIO Ein/Aus
 * - hal_i2c.h      : I2C Bus Kommunikation
 * - hal_spi.h      : SPI Bus Kommunikation
 * - hal_adc.h      : Analog zu Digital Konvertierung
 * - hal_pwm.h      : PWM Motor Control
 * - hal_uart.h     : Serielle Kommunikation
 * - hal_can.h      : CAN-Bus Kommunikation
 */

// Config (muss zuerst importiert werden)
#include "hal_config.h"

// GPIO & LED
#include "hal_gpio.h"

// I2C (für IMU, Barometer)
#include "hal_i2c.h"

// SPI (für Drucksensor, SD-Card)
#include "hal_spi.h"

// ADC (für Weight Sensors)
#include "hal_adc.h"

// PWM (für Motor)
#include "hal_pwm.h"

// UART (für Telemetry Serial)
#include "hal_uart.h"

// CAN (für STM32 Roboterarm)
#include "hal_can.h"

/**
 * Zentrale HAL-Initialisierung
 * Ruft alle HAL-Module auf einmal auf
 * 
 * @return true wenn alle erfolgreich, false wenn Fehler
 */
namespace HAL {
    bool initializeAll();
}
