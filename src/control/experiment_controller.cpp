#include "control/experiment_controller.h"

ExperimentController::ExperimentController() {
    // Konstruktor
}

ExperimentController::~ExperimentController() {
    // Destruktor
}

bool ExperimentController::init() {
    Serial.println("INFO: Initializing experiment controller");
    current_state = ExperimentState::IDLE;
    return true;
}

bool ExperimentController::start() {
    if (current_state != ExperimentState::IDLE && current_state != ExperimentState::ARMED) {
        Serial.println("ERROR: Cannot start from current state");
        return false;
    }
    
    onStateChange(ExperimentState::RUNNING);
    start_time = millis();
    total_paused_time = 0;
    
    Serial.println("INFO: Experiment started");
    return true;
}

bool ExperimentController::pause() {
    if (current_state != ExperimentState::RUNNING) {
        return false;
    }
    
    onStateChange(ExperimentState::PAUSED);
    pause_start_time = millis();
    
    Serial.println("INFO: Experiment paused");
    return true;
}

bool ExperimentController::resume() {
    if (current_state != ExperimentState::PAUSED) {
        return false;
    }
    
    total_paused_time += (millis() - pause_start_time);
    onStateChange(ExperimentState::RUNNING);
    
    Serial.println("INFO: Experiment resumed");
    return true;
}

void ExperimentController::reset() {
    onStateChange(ExperimentState::IDLE);
    start_time = 0;
    pause_start_time = 0;
    total_paused_time = 0;
    
    Serial.println("INFO: Experiment reset");
}

void ExperimentController::update() {
    if (current_state != ExperimentState::RUNNING) {
        return;
    }
    
    uint32_t elapsed = millis() - start_time - total_paused_time;
    
    // Prüfe auf Max-Dauer überschritten
    if (elapsed > MAX_EXPERIMENT_DURATION) {
        onStateChange(ExperimentState::FINISHED);
        Serial.println("INFO: Experiment finished (max duration reached)");
    }
    
    // TODO: Implementiere spezifische Experiment-Logik
    // - Steuere externe Geräte basierend auf Zeit/Sensoren
    // - Prüfe auf Fehler-Bedingungen
    // - Führe Sicherheits-Checks durch
}

ExperimentState ExperimentController::getState() const {
    return current_state;
}

uint32_t ExperimentController::getDuration() const {
    if (current_state == ExperimentState::IDLE) return 0;
    
    if (current_state == ExperimentState::RUNNING) {
        return millis() - start_time - total_paused_time;
    }
    
    return millis() - start_time - total_paused_time;
}

void ExperimentController::calibrateSensors() {
    Serial.println("INFO: Starting sensor calibration");
    onStateChange(ExperimentState::CALIBRATING);
    
    // TODO: Rufe Kalibrierroutinen auf
    
    onStateChange(ExperimentState::IDLE);
    Serial.println("INFO: Calibration complete");
}

bool ExperimentController::activateDevice(uint8_t device_id) {
    // TODO: Implementiere Geräte-Aktivierung
    // - Steuere GPIO-Pins
    // - Schreibe zu Ventilen/Pumpen/Relais
    
    Serial.printf("INFO: Device %d activated\n", device_id);
    return true;
}

bool ExperimentController::deactivateDevice(uint8_t device_id) {
    // TODO: Implementiere Geräte-Deaktivierung
    
    Serial.printf("INFO: Device %d deactivated\n", device_id);
    return true;
}

void ExperimentController::onStateChange(ExperimentState new_state) {
    if (current_state == new_state) return;
    
    current_state = new_state;
    
    Serial.print("INFO: State changed to: ");
    switch (new_state) {
        case ExperimentState::IDLE:
            Serial.println("IDLE");
            break;
        case ExperimentState::ARMED:
            Serial.println("ARMED");
            break;
        case ExperimentState::RUNNING:
            Serial.println("RUNNING");
            break;
        case ExperimentState::CALIBRATING:
            Serial.println("CALIBRATING");
            break;
        case ExperimentState::PAUSED:
            Serial.println("PAUSED");
            break;
        case ExperimentState::FINISHED:
            Serial.println("FINISHED");
            break;
        case ExperimentState::ERROR:
            Serial.println("ERROR");
            break;
    }
}
