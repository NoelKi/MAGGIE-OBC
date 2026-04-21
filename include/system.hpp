#pragma once

#include "control/mission_control.hpp"
#include "services/logging_service.hpp"

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
    
    /**
     * @brief Gibt Mission-Kontroller zurück
     */
    MissionControl* getMissionControl();
    
    /**
     * @brief Gibt Logging-Service zurück
     */
    LoggingService* getLoggingService();
    
    /**
     * @brief Gibt Aufwärts-Zeit zurück
     */
    uint32_t getUptimeMs() const;

private:
    MissionControl mission_control;
    LoggingService logger;
    uint32_t startup_time = 0;
    bool system_healthy = false;
    
    void printWelcomeBanner();
};

// Externe System-Instanz (definiert in main.cpp)
extern System* g_system;