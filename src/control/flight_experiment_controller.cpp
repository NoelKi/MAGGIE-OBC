#include "control/flight_experiment_controller.h"

FlightExperimentController::FlightExperimentController() {
    // Konstruktor
}

FlightExperimentController::~FlightExperimentController() {
    // Destruktor
}

bool FlightExperimentController::init() {
    Serial.println("===================================");
    Serial.println("MAGGIE Flight Experiment Controller");
    Serial.println("Initializing...");
    Serial.println("===================================\n");
    
    current_state = FlightExperimentState::IDLE;
    current_flight_phase = FlightPhase::GROUND;
    error_flag = false;
    error_code = 0;
    
    Serial.println("INFO: Flight Experiment Controller initialized");
    return true;
}

void FlightExperimentController::update() {
    // Prüfe auf Fehler
    checkSafetyConditions();
    
    // Handle current state
    switch (current_state) {
        case FlightExperimentState::IDLE:
            // Warte auf Arming
            break;
            
        case FlightExperimentState::ARMED:
            // Prüfe auf Lift-Off Signal
            // Normalerweise wird das von MissionControl erkannt
            break;
            
        case FlightExperimentState::PRE_LAUNCH:
            // 10 Sekunden vor Lift-Off
            // Könnte letzte Sensor-Checks durchführen
            break;
            
        case FlightExperimentState::RUNNING:
            // Experiment läuft
            executeExperimentSequence();
            logExperimentData();
            break;
            
        case FlightExperimentState::PAUSED:
            // Experiment pausiert (z.B. bei Fehler)
            // Warte auf Resume oder Abort
            break;
            
        case FlightExperimentState::EMERGENCY_STOP:
            // Notfall - Alle Systeme sperren
            break;
            
        case FlightExperimentState::FINISHED:
            // Experiment beendet, nur noch Datenerfassung
            logExperimentData();
            break;
            
        case FlightExperimentState::ERROR:
            // Fehler - sichere Daten und benachrichtige MissionControl
            break;
    }
}

bool FlightExperimentController::armExperiment() {
    if (current_state != FlightExperimentState::IDLE) {
        Serial.println("ERROR: Cannot arm from current state");
        return false;
    }
    
    Serial.println("INFO: Arming experiment...");
    
    // Validiere Sensoren
    if (!validateSensors()) {
        handleError(0x01, "Sensor validation failed during arming");
        return false;
    }
    
    current_state = FlightExperimentState::ARMED;
    Serial.println("INFO: Experiment armed and ready");
    return true;
}

void FlightExperimentController::startExperiment() {
    if (current_state != FlightExperimentState::PRE_LAUNCH && 
        current_state != FlightExperimentState::ARMED) {
        Serial.println("ERROR: Cannot start from current state");
        return;
    }
    
    experiment_start_time = millis();
    current_state = FlightExperimentState::RUNNING;
    Serial.println("INFO: Experiment started");
}

void FlightExperimentController::pauseExperiment() {
    if (current_state != FlightExperimentState::RUNNING) {
        return;
    }
    
    current_state = FlightExperimentState::PAUSED;
    Serial.println("INFO: Experiment paused");
}

void FlightExperimentController::resumeExperiment() {
    if (current_state != FlightExperimentState::PAUSED) {
        return;
    }
    
    current_state = FlightExperimentState::RUNNING;
    Serial.println("INFO: Experiment resumed");
}

void FlightExperimentController::emergencyStop(const char* reason) {
    Serial.printf("EMERGENCY: Emergency stop triggered - Reason: %s\n", reason);
    
    current_state = FlightExperimentState::EMERGENCY_STOP;
    error_flag = true;
    
    // TODO: Implementiere schnelle Abschaltung aller Geräte
    // - Schließe alle Ventile
    // - Stoppe alle Motoren
    // - Sichere kritische Daten
}

void FlightExperimentController::onFlightPhaseChange(FlightPhase new_phase) {
    if (current_flight_phase == new_phase) {
        return;
    }
    
    current_flight_phase = new_phase;
    
    Serial.print("INFO: Flight phase changed to: ");
    switch (new_phase) {
        case FlightPhase::GROUND:
            Serial.println("GROUND");
            break;
        case FlightPhase::ASCENT:
            Serial.println("ASCENT");
            handleAscentPhase();
            break;
        case FlightPhase::MICROGRAVITY:
            Serial.println("MICROGRAVITY");
            handleMicrogravityPhase();
            break;
        case FlightPhase::DESCENT:
            Serial.println("DESCENT");
            handleDescentPhase();
            break;
        case FlightPhase::RECOVERY:
            Serial.println("RECOVERY");
            handleRecoveryPhase();
            break;
    }
}

FlightExperimentState FlightExperimentController::getState() const {
    return current_state;
}

FlightPhase FlightExperimentController::getCurrentFlightPhase() const {
    return current_flight_phase;
}

uint32_t FlightExperimentController::getExperimentDuration() const {
    if (current_state == FlightExperimentState::IDLE) {
        return 0;
    }
    return millis() - experiment_start_time;
}

bool FlightExperimentController::hasError() const {
    return error_flag;
}

uint16_t FlightExperimentController::getErrorCode() const {
    return error_code;
}

void FlightExperimentController::handleAscentPhase() {
    Serial.println("  - Experiment suspended during ascent");
    pauseExperiment();
}

void FlightExperimentController::handleMicrogravityPhase() {
    Serial.println("  - Starting experiment (MICROGRAVITY)");
    if (current_state == FlightExperimentState::ARMED || 
        current_state == FlightExperimentState::PAUSED) {
        startExperiment();
    }
}

void FlightExperimentController::handleDescentPhase() {
    Serial.println("  - Suspending experiment (descent phase)");
    pauseExperiment();
}

void FlightExperimentController::handleRecoveryPhase() {
    Serial.println("  - Finalizing experiment data");
    if (current_state == FlightExperimentState::RUNNING) {
        current_state = FlightExperimentState::FINISHED;
    }
}

void FlightExperimentController::executeExperimentSequence() {
    // TODO: Implementiere Experiment-Logik basierend auf Flugphase
    // - Stelle Ventile ein
    // - Steuere Geräte
    // - Prüfe Sensoren
    // - Sammle Daten
}

void FlightExperimentController::checkSafetyConditions() {
    // TODO: Implementiere Safety-Checks
    // - Prüfe Temperaturgrenzen
    // - Prüfe Druckgrenzen
    // - Prüfe Stromaufnahme
    // Bei Fehler: handleError()
}

void FlightExperimentController::logExperimentData() {
    // TODO: Implementiere Daten-Logging
    // - Schreibe zu SD-Karte
    // - Übertrage via Telemetrie
}

void FlightExperimentController::handleError(uint16_t code, const char* message) {
    error_flag = true;
    error_code = code;
    current_state = FlightExperimentState::ERROR;
    
    Serial.printf("ERROR [0x%04X]: %s\n", code, message);
    
    // Pause experiment auf Fehler
    if (current_state != FlightExperimentState::FINISHED) {
        pauseExperiment();
    }
}

bool FlightExperimentController::validateSensors() const {
    // TODO: Implementiere Sensor-Validierung für Pre-Flight
    return true;
}
