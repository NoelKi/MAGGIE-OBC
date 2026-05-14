#pragma once
#include <cstdint>
#include "drivers/weight_sensor_driver.hpp"
#include "hal/force_sensor_hal.hpp"
#include "hal/sensor_hal.hpp"
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

    uint32_t last_weight_read_ms_ = 0;      ///< Zeitstempel des letzten Weight-Auslesens
    uint32_t last_force_read_ms_ = 0;       ///< Zeitstempel des letzten Force-Auslesens

    void printWelcomeBanner();
    void handleWeightReading();
    void handleForceReading();
};

// Externe System-Instanz (definiert in main.cpp)
extern System* g_system;