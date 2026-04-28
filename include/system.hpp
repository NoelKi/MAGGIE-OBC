#pragma once
#include <cstdint>
#include "drivers/weight_sensor_driver.hpp"
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
    WeightSensorDriver* weight_sensor_ = nullptr;

    uint32_t last_weight_read_ms_ = 0;  ///< Zeitstempel des letzten Auslesens

    void printWelcomeBanner();
    void handleWeightReading();
};

// Externe System-Instanz (definiert in main.cpp)
extern System* g_system;