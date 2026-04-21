#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Flight Experiment Controller
 * 
 * Eigenständiger Controller für das REXUS-Flugexperiment.
 * Erkennt Flugphasen SELBSTSTÄNDIG anhand von Sensordaten und
 * steuert das Experiment entsprechend.
 * 
 * Verantwortlich für:
 * - Flugphasen-Erkennung (Liftoff, Apogee, Descent, Landing)
 * - Experiment-Start/Stop basierend auf Phase
 * - Datensammlung während Mikrogravitation
 * - Sichere Abschaltung bei Landung
 * 
 * MissionControl ruft feedSensorData() + update() auf,
 * der Rest passiert intern.
 */

// ============================================================================
// Experiment State Machine
// ============================================================================

enum class FlightExperimentState {
    IDLE,                  // Nicht initialisiert
    ARMED,                 // Bereit — wartet auf Liftoff
    ASCENT_MONITORING,     // Aufstieg erkannt — Sensoren aktiv
    EXPERIMENT_ACTIVE,     // Mikrogravitation — Experiment läuft
    DESCENT_SAFING,        // Abstieg — Experiment gestoppt, Daten sichern
    RECOVERY_DATA_SAVE,    // Landung — finale Datensicherung
    FINISHED,              // Alles abgeschlossen
    ERROR,                 // Fehler aufgetreten
};

// ============================================================================
// Flight Phases (intern erkannt)
// ============================================================================

enum class FlightPhase {
    GROUND,            // Am Boden
    ASCENT,            // Aufstieg (hohe Beschleunigung)
    MICROGRAVITY,      // Schwerelosigkeit (nahe 0g)
    DESCENT,           // Abstieg (Fallschirm, Beschleunigung steigt)
    RECOVERY,          // Gelandet (1g + niedrige Höhe)
};

// ============================================================================
// Controller
// ============================================================================

class FlightExperimentController {
public:
    FlightExperimentController();
    ~FlightExperimentController();

    bool init();
    void update();

    // === Von MissionControl aufgerufen ===

    /** Arme das Experiment (Transition IDLE → ARMED) */
    bool armExperiment();

    /** Sensordaten einspeisen — wird jeden Zyklus aufgerufen */
    void feedSensorData(float accel_x, float accel_y, float accel_z,
                        float pressure, float altitude);

    /** Notfall-Stopp (alle Aktoren aus, Daten sichern) */
    void emergencyStop(const char* reason);

    // === Getters ===

    FlightExperimentState getState() const;
    FlightPhase getCurrentFlightPhase() const;
    uint32_t getExperimentDuration() const;
    float getMaxAltitude() const;
    bool hasError() const;
    uint16_t getErrorCode() const;

private:
    // State
    FlightExperimentState current_state = FlightExperimentState::IDLE;
    FlightPhase current_phase = FlightPhase::GROUND;

    // Timing
    uint32_t arm_time = 0;
    uint32_t experiment_start_time = 0;
    uint32_t phase_entry_time = 0;
    uint32_t microgravity_start_time = 0;

    // Sensordaten (zuletzt empfangen)
    float last_accel_x = 0.0f;
    float last_accel_y = 0.0f;
    float last_accel_z = 0.0f;
    float last_pressure = 0.0f;
    float last_altitude = 0.0f;
    float max_altitude = 0.0f;

    // Fehler
    bool error_flag = false;
    uint16_t error_code = 0;

    // === Schwellwerte für Phasenerkennung ===
    static constexpr float LIFTOFF_ACCEL_THRESHOLD  = 20.0f;   // m/s² (>2g)
    static constexpr float APOGEE_ACCEL_THRESHOLD   = 2.0f;    // m/s² (nahe 0g)
    static constexpr float DESCENT_ACCEL_THRESHOLD   = 15.0f;   // m/s² (Fallschirm-Öffnung)
    static constexpr float LANDING_ACCEL_LOW         = 9.0f;    // m/s² (~1g)
    static constexpr float LANDING_ACCEL_HIGH        = 11.0f;   // m/s²
    static constexpr float LANDING_ALT_THRESHOLD     = 500.0f;  // Meter

    // Zeitkonstanten
    static constexpr uint32_t PHASE_CONFIRM_TIME     = 500;     // ms — Phase muss 500ms stabil sein
    static constexpr uint32_t MAX_EXPERIMENT_TIME     = 120000;  // ms — max 2min Mikrogravitation
    static constexpr uint32_t RECOVERY_SAVE_TIME      = 30000;   // ms — 30s Daten sichern

    // === Phasenerkennung (intern) ===
    bool detectLiftOff() const;
    bool detectApogee() const;
    bool detectDescent() const;
    bool detectLanding() const;
    float getAccelMagnitude() const;

    // === Phase Handler ===
    void handleArmed();
    void handleAscentMonitoring();
    void handleExperimentActive();
    void handleDescentSafing();
    void handleRecoveryDataSave();

    // === Experiment-Aktionen ===
    void startExperimentActions();
    void stopExperimentActions();
    void saveFlightData();

    // === Phase Transition ===
    void transitionToPhase(FlightPhase new_phase);
    void transitionToState(FlightExperimentState new_state);
    const char* stateToString(FlightExperimentState state) const;
    const char* phaseToString(FlightPhase phase) const;

    // Fehlerbehandlung
    void handleError(uint16_t code, const char* message);
};
