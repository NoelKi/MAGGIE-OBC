#pragma once

#include <Arduino.h>
#include <cstdint>
#include "services/data_collection_service.h"
#include "services/telemetry_service.h"
#include "control/safety_monitor.h"
#include "control/flight_experiment_controller.h"

/**
 * @brief Zentrale Mission-Kontrolleinheit
 * Koordiniert alle Subsysteme und verwaltet Mission-Ablauf
 */

enum class MissionState {
    STARTUP,          // Boot-Phase
    PREFLIGHT_CHECK,  // Vor-Flug-Prüfung
    STANDBY,          // Wartend auf Start-Signal
    ASCENT,           // Raketenstartphase
    MICROGRAVITY,     // Schwerelosigkeits-Phase
    DESCENT,          // Abstiegsphase
    RECOVERY,         // Bergungsphase
    SHUTDOWN,         // Herunterfahren
};

class MissionControl {
public:
    MissionControl();
    ~MissionControl();
    
    /**
     * @brief Initialisiert alle Subsysteme
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Hauptkontroll-Loop (sollte in loop() aufgerufen werden)
     */
    void run();
    
    /**
     * @brief Gibt aktuellen Mission-Status zurück
     */
    MissionState getCurrentState() const;
    
    /**
     * @brief Gibt Mission-Zeit zurück (seit Start)
     */
    uint32_t getMissionTime() const;
    
    /**
     * @brief Führt Vor-Flug-Checks durch
     * @return true wenn alle Checks erfolgreich
     */
    bool preflightCheck();
    
    /**
     * @brief Startet Mission
     */
    void startMission();

    /**
     * @brief Prüft ob Start-Signal empfangen wurde
     * @return true wenn Start-Signal empfangen wurde
     */
    bool isLiftOffSignal();
    
    /**
     * @brief Stoppt Mission (Notfall)
     */
    void abortMission();

private:
    MissionState current_state = MissionState::STARTUP;
    uint32_t mission_start_time = 0;
    uint32_t last_telemetry_time = 0;
    
    // Subsystem-Instances
    DataCollectionService data_collector;
    TelemetryService telemetry;
    SafetyMonitor safety;
    FlightExperimentController experiment;
    
    // Telemetrie-Buffer
    TelemetryData last_telemetry = {};
    char telemetry_buffer[256];
    
    static constexpr uint32_t TELEMETRY_INTERVAL = 100;  // 100ms = 10Hz
    
    // Flugphasen-Erkennung
    bool detectApogee();
    bool detectDescent();
    bool detectLanding();
    
    void onStateChange(MissionState new_state);
    void printMissionStatus();
};
