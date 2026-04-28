#pragma once
#include <cstdint>

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
    
    void printWelcomeBanner();
};

// Externe System-Instanz (definiert in main.cpp)
extern System* g_system;