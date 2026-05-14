/**
 * @file HAL_OVERVIEW.md
 * @brief Overview of all Hardware Abstraction Layer modules
 * 
 * This document provides a complete overview of the HAL structure
 * for the MAGGIE OBC (On-Board Computer) running on Teensy 4.1
 */

# MAGGIE OBC - Hardware Abstraction Layer (HAL) Overview

## Module Structure

### Core HAL Modules

#### 1. **Force Sensor HAL** (`force_sensor_hal.hpp/cpp`)
- **Purpose**: Interface for custom analog force sensors (3-axis)
- **Pins Used**: 
  - X-axis: PIN 30
  - Y-axis: PIN 31
  - Z-axis: PIN 32
- **Features**:
  - Analog input reading
  - Offset calibration
  - Direct force measurement conversion
- **Data Structure**: `ForceSensorReading`

#### 2. **Motor HAL** (`motor_hal.hpp/cpp`)
- **Purpose**: PWM control for DRV8871 motor drivers
- **Motors Supported**: 3 independent motors
- **Motor Pins**:
  - Motor 1: Pins 15 (A), 14 (B)
  - Motor 2: Pins 23 (A), 22 (B)
  - Motor 3: Pins 18 (A), 19 (B)
- **Features**:
  - Bidirectional speed control (-255 to +255)
  - Individual motor instances
  - PWM frequency configuration

#### 3. **LED HAL** (`led_hal.hpp/cpp`)
- **Purpose**: Status LED control with PWM brightness
- **LED Pins**:
  - LED 1: Pin 5
  - LED 2: Pin 6
- **Features**:
  - Brightness control (0-255)
  - On/Off/Toggle functions
  - Blinking support

#### 4. **UART HAL** (`uart_hal.hpp/cpp`)
- **Purpose**: Serial communication interface
- **ARM Communication**: Pins 0 (TX), 1 (RX)
- **Features**:
  - Configurable baud rate
  - Send/receive operations
  - Data availability check

#### 5. **Camera HAL** (`camera_hal.hpp/cpp`)
- **Purpose**: Camera module communication via UART
- **Camera Pins**:
  - Camera 1: TX=7, RX=8 (UART2)
  - Camera 2 (Backup): RX=24
  - Camera 3: TX=25
- **Features**:
  - Individual camera control
  - Command send/receive
  - Image capture interface

#### 6. **IMU HAL** (`imu_hal.hpp/cpp`)
- **Purpose**: Accelerometer and Gyroscope interface
- **Communication**: SPI
- **Chip Select Pins**:
  - Accelerometer: Pin 37
  - Gyroscope: Pin 36
- **SPI Pins**: MOSI=11, MISO=12, SCK=13
- **Features**:
  - 6-axis IMU data (3-axis accel + 3-axis gyro)
  - Calibration support
  - Self-test capability
- **Data Structure**: `IMUReading`

#### 7. **REXUS HAL** (`rexus_hal.hpp/cpp`)
- **Purpose**: REXUS experiment signal interface
- **Signal Pins**:
  - L0_t (Launch): Pin 40
  - SOE_i (Start of Experiment): Pin 39
  - SODS_i (Start of Descent): Pin 38
- **Features**:
  - Launch detection
  - Experiment phase tracking
  - Descent detection

#### 8. **Communication HAL** (`communication_hal.hpp/cpp`)
- **Purpose**: Up/Down-link signal monitoring
- **Signal Pins**:
  - Uplink Plus: Pin 34
  - Uplink Minus: Pin 35
- **Features**:
  - Uplink signal detection
  - Differential signal reading

### Supporting Files

#### **Weight Sensor** (HX711 - Already Exists)
- **Sensor**: Purchased scale with HX711 amplifier
- **Pins**:
  - DOUT: Pin 2 (mapped from PIN_FORCE_X_1)
  - SCK: Pin 3 (mapped from PIN_FORCE_Y_1)

#### **Sensor HAL Configuration** (`sensor_hal.hpp`)
- Central configuration file
- Maps all pin definitions to hardware interfaces
- Includes all HAL module headers

## System Integration

### System Class (`system.hpp/cpp`)

The `System` class integrates all HAL modules:

```cpp
class System {
    WeightSensorDriver* weight_sensor_;    // HX711 weight
    ForceSensorHAL* force_sensor_2_;       // Custom force sensor
    
    // Additional HAL instances would be added here:
    // MotorHAL* motor_1_;
    // MotorHAL* motor_2_;
    // MotorHAL* motor_3_;
    // LEDHAL* led_1_;
    // LEDHAL* led_2_;
    // IMUHAL* imu_;
    // REXUSHAL* rexus_;
    // CommunicationHAL* comm_;
    // CameraHAL* camera_1_;
    // CameraHAL* camera_2_;
    // CameraHAL* camera_3_;
};
```

## Pin Mapping Summary

| Component | Pin | Function | HAL Module |
|-----------|-----|----------|-----------|
| ARM TX | 0 | UART1 TX | UART_HAL |
| ARM RX | 1 | UART1 RX | UART_HAL |
| HX711 DOUT | 2 | Weight Data | HX711 (existing) |
| HX711 SCK | 3 | Weight Clock | HX711 (existing) |
| Force X | 30 | Force X-axis | ForceSensorHAL |
| Force Y | 31 | Force Y-axis | ForceSensorHAL |
| Force Z | 32 | Force Z-axis | ForceSensorHAL |
| Motor 1 A | 15 | Motor 1 Channel A | MotorHAL |
| Motor 1 B | 14 | Motor 1 Channel B | MotorHAL |
| Motor 2 A | 23 | Motor 2 Channel A | MotorHAL |
| Motor 2 B | 22 | Motor 2 Channel B | MotorHAL |
| Motor 3 A | 18 | Motor 3 Channel A | MotorHAL |
| Motor 3 B | 19 | Motor 3 Channel B | MotorHAL |
| LED 1 | 5 | Status LED 1 | LEDHAL |
| LED 2 | 6 | Status LED 2 | LEDHAL |
| Camera 1 TX | 7 | Camera 1 TX | CameraHAL |
| Camera 1 RX | 8 | Camera 1 RX | CameraHAL |
| Force CLK 2 | 9 | Force Sensor Control | (Reserved) |
| Chip Select Force | 10 | Force CS | (Reserved) |
| SPI MOSI | 11 | SPI Data Out | IMU_HAL |
| SPI MISO | 12 | SPI Data In | IMU_HAL |
| SPI SCK | 13 | SPI Clock | IMU_HAL |
| CS Gyro | 36 | Gyroscope CS | IMU_HAL |
| CS Accel | 37 | Accelerometer CS | IMU_HAL |
| SODS_i | 38 | Descent Signal | REXUS_HAL |
| SOE_i | 39 | Experiment Start | REXUS_HAL |
| L0_t | 40 | Launch Signal | REXUS_HAL |
| Uplink Minus | 35 | Downlink Signal - | Communication_HAL |
| Uplink Plus | 34 | Downlink Signal + | Communication_HAL |
| Camera 3 TX | 25 | Camera 3 TX | CameraHAL |
| Camera Backup RX | 24 | Camera Backup RX | CameraHAL |
| Temp CS | 26 | Temperature Sensor CS | (Reserved) |

## Usage Example

```cpp
// Initialize system with all HAL modules
System system;
if (!system.init()) {
    Serial.println("System initialization failed!");
    return;
}

// Main loop - system handles periodic sensor reads
while (true) {
    system.run();
    delay(10);
}
```

## Future Enhancements

- Actual IMU driver implementation (currently placeholder)
- Temperature sensor HAL
- Advanced SPI communication
- Real hardware UART implementation for cameras
- Motor speed feedback (encoder support)
- LED patterns and animations
- Event-based architecture for REXUS signals

---
Generated: MAGGIE OBC v1.6
Author: Kieran Mai
Date: 2026-05-14
