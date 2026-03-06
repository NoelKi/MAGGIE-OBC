#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Flight Experiment Controller
 * Steuert das echte Experiment während des Raketenflugs
 * 
 * Funktionen:
 * - Steuert Experiment basierend auf Flugphase
 * - Sensorgesteuerte Experiment-Logik
 * - Echtzeit-Datenerfassung
 * - Safety-Integrationen
 * - Fehlerbehandlung im Flug
 */

enum class FlightExperimentState {
    IDLE,              // Vor Start
    ARMED,             // Bereit für Start
    PRE_LAUNCH,        // 10 Sekunden vor Start
    RUNNING,           // Experiment läuft
    PAUSED,            // Pausiert (z.B. bei Fehler)
    EMERGENCY_STOP,    // Notfall-Abschaltung
    FINISHED,          // Abgeschlossen
    ERROR,             // Fehler aufgetreten
};

/**
 * Flugphasen für Experiment-Steuerung
 */
enum class FlightPhase {
    GROUND,            // Am Boden
    ASCENT,            // Aufstieg
    MICROGRAVITY,      // Schwerelosigkeit (Apogee)
    DESCENT,           // Abstieg
    RECOVERY,          // Bergung
};

class FlightExperimentController {
public:
    FlightExperimentController();
    ~FlightExperimentController();
    
    /**
     * @brief Initialisiert Flight Experiment Controller
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Update-Loop (sollte häufig aufgerufen werden)
     */
    void update();
    
    /**
     * @brief Arme Experiment für Start
     * @return true wenn erfolgreich gearmt
     */
    bool armExperiment();
    
    /**
     * @brief Starte Experiment (normalerweise automatisch nach Lift-Off)
     */
    void startExperiment();
    
    /**
     * @brief Pausiere Experiment (bei Fehler)
     */
    void pauseExperiment();
    
    /**
     * @brief Setze Experiment fort
     */
    void resumeExperiment();
    
    /**
     * @brief Notfall-Stopp (schnelle Abschaltung aller Geräte)
     * @param reason Grund für Notfall-Stopp (wird geloggt)
     */
    void emergencyStop(const char* reason);
    
    /**
     * @brief Benachrichtige über Flugphase-Änderung
     * @param new_phase Neue Flugphase
     */
    void onFlightPhaseChange(FlightPhase new_phase);
    
    /**
     * @brief Gibt aktuellen Experiment-Status zurück
     */
    FlightExperimentState getState() const;
    
    /**
     * @brief Gibt aktuelle Flugphase zurück
     */
    FlightPhase getCurrentFlightPhase() const;
    
    /**
     * @brief Gibt Experiment-Dauer zurück
     */
    uint32_t getExperimentDuration() const;
    
    /**
     * @brief Prüfe auf kritische Fehler
     * @return true wenn Fehler erkannt
     */
    bool hasError() const;

    /**
     * @brief Prüfe auf kritische Fehler
     * @return true wenn Fehler erkannt
     */
    bool validateSensors() const;
    
    /**
     * @brief Gibt Fehler-Code zurück
     */
    uint16_t getErrorCode() const;

private:
    FlightExperimentState current_state = FlightExperimentState::IDLE;
    FlightPhase current_flight_phase = FlightPhase::GROUND;
    
    uint32_t experiment_start_time = 0;
    uint32_t pre_launch_warning_time = 0;
    
    bool error_flag = false;
    uint16_t error_code = 0;
    
    // Phase-spezifische Logik
    void handleAscentPhase();
    void handleMicrogravityPhase();
    void handleDescentPhase();
    void handleRecoveryPhase();
    
    // Experiment-Logik
    void executeExperimentSequence();
    void checkSafetyConditions();
    void logExperimentData();
    
    // Fehlerbehandlung
    void handleError(uint16_t error_code, const char* message);
};
