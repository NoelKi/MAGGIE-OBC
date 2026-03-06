#include "control/flight_experiment_controller.h"
#include <cmath>

FlightExperimentController::FlightExperimentController() {}
FlightExperimentController::~FlightExperimentController() {}

// ============================================================================
// Initialisierung
// ============================================================================

bool FlightExperimentController::init() {
    Serial.println("INFO: FlightExperimentController init");

    current_state = FlightExperimentState::IDLE;
    current_phase = FlightPhase::GROUND;

    last_accel_x = 0.0f;
    last_accel_y = 0.0f;
    last_accel_z = 0.0f;
    last_pressure = 0.0f;
    last_altitude = 0.0f;
    max_altitude = 0.0f;

    error_flag = false;
    error_code = 0;

    return true;
}

// ============================================================================
// Sensordaten einspeisen
// ============================================================================

void FlightExperimentController::feedSensorData(float accel_x, float accel_y, float accel_z,
                                                  float pressure, float altitude) {
    last_accel_x = accel_x;
    last_accel_y = accel_y;
    last_accel_z = accel_z;
    last_pressure = pressure;
    last_altitude = altitude;

    // Max-Altitude tracken
    if (altitude > max_altitude) {
        max_altitude = altitude;
    }
}

// ============================================================================
// Haupt-Update (State Machine)
// ============================================================================

void FlightExperimentController::update() {
    switch (current_state) {
        case FlightExperimentState::IDLE:
            // Nichts — wartet auf armExperiment()
            break;

        case FlightExperimentState::ARMED:
            handleArmed();
            break;

        case FlightExperimentState::ASCENT_MONITORING:
            handleAscentMonitoring();
            break;

        case FlightExperimentState::EXPERIMENT_ACTIVE:
            handleExperimentActive();
            break;

        case FlightExperimentState::DESCENT_SAFING:
            handleDescentSafing();
            break;

        case FlightExperimentState::RECOVERY_DATA_SAVE:
            handleRecoveryDataSave();
            break;

        case FlightExperimentState::FINISHED:
        case FlightExperimentState::ERROR:
            // Terminal-Zustände — nichts tun
            break;
    }
}

// ============================================================================
// Arming
// ============================================================================

bool FlightExperimentController::armExperiment() {
    if (current_state != FlightExperimentState::IDLE) {
        Serial.println("ERROR: Cannot arm — not in IDLE state");
        return false;
    }

    arm_time = millis();
    transitionToState(FlightExperimentState::ARMED);
    Serial.println("INFO: Experiment ARMED — waiting for liftoff");
    return true;
}

// ============================================================================
// Phase Handler
// ============================================================================

void FlightExperimentController::handleArmed() {
    // Warte auf Liftoff-Erkennung
    if (detectLiftOff()) {
        Serial.println(">>> LIFTOFF detected!");
        transitionToPhase(FlightPhase::ASCENT);
        transitionToState(FlightExperimentState::ASCENT_MONITORING);
    }
}

void FlightExperimentController::handleAscentMonitoring() {
    // Aufstiegsphase — Experiment ist noch NICHT aktiv
    // Warte auf Schwerelosigkeit (Apogee)

    if (detectApogee()) {
        Serial.println(">>> APOGEE detected — entering microgravity!");
        microgravity_start_time = millis();
        transitionToPhase(FlightPhase::MICROGRAVITY);
        transitionToState(FlightExperimentState::EXPERIMENT_ACTIVE);
        startExperimentActions();
    }
}

void FlightExperimentController::handleExperimentActive() {
    // Experiment läuft — Mikrogravitationsphase
    // TODO: Hier die eigentliche Experiment-Logik aufrufen
    //       z.B. Ventile steuern, Kameras, Daten erfassen

    // Timeout-Check: max. Experimentdauer
    if (millis() - microgravity_start_time > MAX_EXPERIMENT_TIME) {
        Serial.println("WARN: Max experiment time reached");
        stopExperimentActions();
        transitionToState(FlightExperimentState::DESCENT_SAFING);
        transitionToPhase(FlightPhase::DESCENT);
        return;
    }

    // Erkennung: Abstieg beginnt
    if (detectDescent()) {
        Serial.println(">>> DESCENT detected — safing experiment");
        stopExperimentActions();
        transitionToPhase(FlightPhase::DESCENT);
        transitionToState(FlightExperimentState::DESCENT_SAFING);
    }
}

void FlightExperimentController::handleDescentSafing() {
    // Abstieg — Experiment gestoppt, Daten sichern

    if (detectLanding()) {
        Serial.println(">>> LANDING detected — saving final data");
        transitionToPhase(FlightPhase::RECOVERY);
        transitionToState(FlightExperimentState::RECOVERY_DATA_SAVE);
    }
}

void FlightExperimentController::handleRecoveryDataSave() {
    // Finale Datensicherung nach Landung
    saveFlightData();

    if (millis() - phase_entry_time > RECOVERY_SAVE_TIME) {
        Serial.println("INFO: Recovery data save complete");
        transitionToState(FlightExperimentState::FINISHED);
    }
}

// ============================================================================
// Flugphasen-Erkennung
// ============================================================================

float FlightExperimentController::getAccelMagnitude() const {
    return sqrtf(last_accel_x * last_accel_x +
                 last_accel_y * last_accel_y +
                 last_accel_z * last_accel_z);
}

bool FlightExperimentController::detectLiftOff() const {
    // Liftoff: Beschleunigung > 20 m/s² (ca. 2g)
    return getAccelMagnitude() > LIFTOFF_ACCEL_THRESHOLD;
}

bool FlightExperimentController::detectApogee() const {
    // Apogee: Beschleunigung nahe 0g (< 2 m/s²)
    // = Schwerelosigkeit beginnt
    return getAccelMagnitude() < APOGEE_ACCEL_THRESHOLD;
}

bool FlightExperimentController::detectDescent() const {
    // Abstieg: Beschleunigung steigt wieder (Fallschirm-Öffnung)
    // UND Höhe sinkt (unter Maximal-Höhe)
    float accel_mag = getAccelMagnitude();
    bool high_accel = accel_mag > DESCENT_ACCEL_THRESHOLD;
    bool below_max = last_altitude < (max_altitude - 100.0f);  // 100m unter Spitze

    return high_accel && below_max;
}

bool FlightExperimentController::detectLanding() const {
    // Landung: ~1g (9-11 m/s²) UND niedrige Höhe (< 500m)
    float accel_mag = getAccelMagnitude();
    bool gravity_normal = (accel_mag > LANDING_ACCEL_LOW && accel_mag < LANDING_ACCEL_HIGH);
    bool low_altitude = (last_altitude < LANDING_ALT_THRESHOLD);

    return gravity_normal && low_altitude;
}

// ============================================================================
// Experiment-Aktionen
// ============================================================================

void FlightExperimentController::startExperimentActions() {
    experiment_start_time = millis();
    Serial.println("INFO: Experiment actions STARTED");

    // TODO: Implementiere experiment-spezifische Aktionen:
    // - Ventile öffnen
    // - Kameras starten
    // - Spezifische Sensoren aktivieren
    // - Motoren/Pumpen einschalten
}

void FlightExperimentController::stopExperimentActions() {
    Serial.printf("INFO: Experiment actions STOPPED (duration: %lu ms)\n",
                  millis() - experiment_start_time);

    // TODO: Implementiere sichere Abschaltung:
    // - Ventile schließen
    // - Motoren stoppen
    // - Kameras stoppen
    // - Alles in sicheren Zustand bringen
}

void FlightExperimentController::saveFlightData() {
    // TODO: Implementiere finale Datensicherung:
    // - SD-Karten-Buffer flushen
    // - Zusammenfassung schreiben
    // - Checksummen berechnen
}

// ============================================================================
// Emergency Stop
// ============================================================================

void FlightExperimentController::emergencyStop(const char* reason) {
    Serial.printf("EMERGENCY STOP: %s\n", reason);

    stopExperimentActions();

    error_flag = true;
    error_code = 0xFFFF;
    transitionToState(FlightExperimentState::ERROR);
}

// ============================================================================
// Getters
// ============================================================================

FlightExperimentState FlightExperimentController::getState() const {
    return current_state;
}

FlightPhase FlightExperimentController::getCurrentFlightPhase() const {
    return current_phase;
}

uint32_t FlightExperimentController::getExperimentDuration() const {
    if (experiment_start_time == 0) return 0;
    return millis() - experiment_start_time;
}

float FlightExperimentController::getMaxAltitude() const {
    return max_altitude;
}

bool FlightExperimentController::hasError() const {
    return error_flag;
}

uint16_t FlightExperimentController::getErrorCode() const {
    return error_code;
}

// ============================================================================
// State Transitions
// ============================================================================

void FlightExperimentController::transitionToPhase(FlightPhase new_phase) {
    if (current_phase == new_phase) return;

    Serial.printf("FLIGHT PHASE: %s → %s\n",
                  phaseToString(current_phase),
                  phaseToString(new_phase));

    current_phase = new_phase;
    phase_entry_time = millis();
}

void FlightExperimentController::transitionToState(FlightExperimentState new_state) {
    if (current_state == new_state) return;

    Serial.printf("EXPERIMENT STATE: %s → %s\n",
                  stateToString(current_state),
                  stateToString(new_state));

    current_state = new_state;
}

const char* FlightExperimentController::stateToString(FlightExperimentState state) const {
    switch (state) {
        case FlightExperimentState::IDLE:                return "IDLE";
        case FlightExperimentState::ARMED:               return "ARMED";
        case FlightExperimentState::ASCENT_MONITORING:    return "ASCENT_MONITORING";
        case FlightExperimentState::EXPERIMENT_ACTIVE:    return "EXPERIMENT_ACTIVE";
        case FlightExperimentState::DESCENT_SAFING:       return "DESCENT_SAFING";
        case FlightExperimentState::RECOVERY_DATA_SAVE:   return "RECOVERY_DATA_SAVE";
        case FlightExperimentState::FINISHED:             return "FINISHED";
        case FlightExperimentState::ERROR:                return "ERROR";
        default:                                          return "UNKNOWN";
    }
}

const char* FlightExperimentController::phaseToString(FlightPhase phase) const {
    switch (phase) {
        case FlightPhase::GROUND:        return "GROUND";
        case FlightPhase::ASCENT:        return "ASCENT";
        case FlightPhase::MICROGRAVITY:  return "MICROGRAVITY";
        case FlightPhase::DESCENT:       return "DESCENT";
        case FlightPhase::RECOVERY:      return "RECOVERY";
        default:                         return "UNKNOWN";
    }
}

// ============================================================================
// Fehlerbehandlung
// ============================================================================

void FlightExperimentController::handleError(uint16_t code, const char* message) {
    error_flag = true;
    error_code = code;
    Serial.printf("ERROR [0x%04X]: %s\n", code, message);

    stopExperimentActions();
    transitionToState(FlightExperimentState::ERROR);
}
