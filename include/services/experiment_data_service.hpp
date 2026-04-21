#pragma once

#include <Arduino.h>
#include <cstdint>
#include "drivers/imu_driver.hpp"
#include "drivers/pressure_temperature_driver.hpp"
#include "drivers/pressure_sensor_driver.hpp"
#include "drivers/weight_sensor_driver.hpp"
#include "drivers/pwm_motor_driver.hpp"
#include "drivers/can_bus_driver.hpp"

/**
 * @brief Experiment-Datenerfassungs-Service (für Roboterarm-Experiment)
 * Koordiniert alle Sensoren für REXUS Flugexperiment mit Roboterarm
 * 
 * Sensoren:
 * - IMU (I2C)
 * - Barometer (I2C)
 * - Drucksensor (SPI)
 * - Zwei Waagen (Gewichtssensoren via ADC)
 * - Motor HDRM (PWM)
 * - CAN-Bus (STM32 Kommunikation)
 */

struct ExperimentTelemetry {
    // IMU Daten
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    
    // Barometer
    float barometer_pressure;
    float barometer_temperature;
    float altitude;
    
    // Drucksensor
    float pressure_sensor;
    float pressure_temp;
    
    // Waagen
    float weight_scale1;
    float weight_scale2;
    float weight_total;
    
    // Motor
    uint8_t motor_speed;
    uint8_t motor_direction;  // 0=stop, 1=forward, 2=backward
    
    // CAN-Bus Status
    bool can_connected;
    
    // Allgemein
    uint32_t timestamp;
    uint32_t sequence;
};

class ExperimentDataService {
public:
    ExperimentDataService();
    ~ExperimentDataService();
    
    /**
     * @brief Initialisiert alle Sensoren und Hardware
     * @return true wenn mindestens kritische Sensoren erfolgreich
     */
    bool init();
    
    /**
     * @brief Sammelt Daten von allen Sensoren
     * @param data Zeiger auf ExperimentTelemetry
     * @return true wenn erfolgreich
     */
    bool collectData(ExperimentTelemetry& data);
    
    /**
     * @brief Sendet Befehl an Roboterarm über CAN-Bus
     * @param command Arm-Befehl
     */
    bool sendArmCommand(const ArmCommand& command);
    
    /**
     * @brief Empfängt Arm-Status von STM32
     * @param status Zeiger auf Arm-Status
     */
    bool receiveArmStatus(ArmStatus& status);
    
    /**
     * @brief Steuert HDRM Motor
     * @param speed Geschwindigkeit 0-100%
     * @param direction 0=stop, 1=forward, 2=backward
     */
    void controlMotor(uint8_t speed, uint8_t direction);
    
    /**
     * @brief Tariert Waagen
     */
    void tareWeightSensors();
    
    /**
     * @brief Gibt Gesamt-Gewicht zurück
     */
    float getTotalWeight() const;
    
    /**
     * @brief Prüft Sensor-Status
     */
    uint8_t getSensorStatus() const;
    
    /**
     * @brief Gibt letzten gesammelten Datenpunkt zurück
     */
    const ExperimentTelemetry& getLastData() const { return last_telemetry; }
    
    /**
     * @brief Gibt Anzahl gesammelter Datenpunkte zurück
     */
    uint32_t getDataPointCount() const { return data_count; }

private:
    // BMI088 CS-Pins (SPI)
    static constexpr uint8_t IMU_ACCEL_CS = 10;  // Chip Select für Accelerometer
    static constexpr uint8_t IMU_GYRO_CS = 9;   // Chip Select für Gyroscope
    
    // Sensoren
    IMUDriver imu;
    PressureTemperatureDriver barometer;
    PressureSensorDriver pressure_sensor;
    WeightSensorDriver weight_sensors;
    PWMMotorDriver motor;
    CANBusDriver can_bus;
    
    // Status
    ExperimentTelemetry last_telemetry = {};
    uint32_t data_count = 0;
    uint32_t last_sample_time = 0;
    uint16_t target_sample_rate = 100;  // 100 Hz
    
    uint8_t getSensorStatusByte() const;
};
