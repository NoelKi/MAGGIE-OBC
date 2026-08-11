#pragma once
#include <cstdint>
#include "drivers/weight_sensor_driver.hpp"
#include "hal/force_sensor_hal.hpp"
#include "hal/sensor_hal.hpp"
#include "hal/imu_hal.hpp"
#include "hal/telemetry_hal.hpp"
#include "camera_config.hpp"
#include "pin_config.hpp"

/**
 * @brief Zentrale System-Klasse
 * Verwaltet den Gesamtzustand des On-Board Computers
 */

class System {
public:
    System();
    ~System();
    
    /**
     * @brief Initialisiert das gesamte System
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Hauptkontroll-Loop
     * Sollte kontinuierlich in loop() aufgerufen werden
     */
    void run();
    

private:
    uint32_t startup_time = 0;
    bool system_healthy = false;

    // -----------------------------------------------------------------------
    // Subsysteme
    // -----------------------------------------------------------------------
    WeightSensorDriver* weight_sensor_ = nullptr;      ///< HX711 Weight Sensor (Sensor 1)
    ForceSensorHAL* force_sensor_2_ = nullptr;         ///< Custom Force Sensor (Sensor 2)
    IMUHAL* imu_ = nullptr;                             ///< BMI088 IMU (SPI)
    TelemetryDownlink* downlink_ = nullptr;            ///< Downlink telemetry (Serial8, pins 34/35)
    CameraHAL* cameras_[CAMERA_COUNT] = {};             ///< siehe camera_config.hpp für die Liste

    uint32_t last_weight_read_ms_ = 0;      ///< Zeitstempel des letzten Weight-Auslesens
    uint32_t last_force_read_ms_ = 0;       ///< Zeitstempel des letzten Force-Auslesens
    uint32_t last_telemetry_ms_ = 0;        ///< Zeitstempel des letzten Telemetrie-Downlinks

    bool imu_ready_ = false;                ///< IMU erfolgreich initialisiert
    bool downlink_ready_ = false;           ///< Downlink-UART bereit

    void printWelcomeBanner();
    void handleWeightReading();
    void handleForceReading();
    void handleTelemetry();
    void handleCameras(uint32_t now_ms);
};

// Externe System-Instanz (definiert in main.cpp)
extern System* g_system;