#include "control/mission_control.h"
#include <cmath>

MissionControl::MissionControl() {
    // Konstruktor
}

MissionControl::~MissionControl() {
    // Destruktor
}

bool MissionControl::init() {
    Serial.println("===================================");
    Serial.println("MAGGIE On-Board Computer");
    Serial.println("Initializing Mission Control...");
    Serial.println("===================================");
    
    // Initialisiere Subsysteme in Reihenfolge
    if (!safety.init()) {
        Serial.println("FATAL: Safety monitor initialization failed");
        return false;
    }
    
    if (!data_collector.init()) {
        Serial.println("WARNING: Data collector initialization failed");
    }
    
    if (!telemetry.init()) {
        Serial.println("WARNING: Telemetry initialization failed");
    }
    
    if (!experiment.init()) {
        Serial.println("FATAL: Experiment controller initialization failed");
        return false;
    }
    
    onStateChange(MissionState::STARTUP);
    Serial.println("INFO: Mission Control initialized successfully");
    
    return true;
}

void MissionControl::run() {
    // Update all subsystems
    safety.update();
    
    // Handle current state
    switch (current_state) {
        case MissionState::STARTUP:
            // Auto transition nach kurzer Zeit
            if (millis() > 5000) {
                onStateChange(MissionState::PREFLIGHT_CHECK);
            }
            break;
            
        case MissionState::PREFLIGHT_CHECK:
            // Automatische Preflight-Checks
            if (preflightCheck()) {
                onStateChange(MissionState::STANDBY);
            }
            break;
            
        case MissionState::STANDBY:
            // Warte auf Experiment-Start
            // TODO: Implementiere Start-Signal-Erkennung
            // z.B. via Kommando über CAN oder Serial
            break;
            
        case MissionState::ASCENT:
        case MissionState::MICROGRAVITY:
        case MissionState::DESCENT:
        case MissionState::RECOVERY:
            // Hauptflug-Phase
            experiment.update();
            
            // Sammle und übertrage Daten
            if (millis() - last_telemetry_time > TELEMETRY_INTERVAL) {
                if (data_collector.collectData(last_telemetry)) {
                    // Berechne Beschleunigung-Magnitude
                    float accel_mag = sqrtf(last_telemetry.accel_x * last_telemetry.accel_x +
                                           last_telemetry.accel_y * last_telemetry.accel_y +
                                           last_telemetry.accel_z * last_telemetry.accel_z);
                    
                    // Sicherheits-Check
                    safety.checkSensorBounds(accel_mag, last_telemetry.temperature, last_telemetry.altitude);
                    
                    // Formatiere und sende Telemetrie
                    telemetry.formatTelemetryCSV(telemetry_buffer, sizeof(telemetry_buffer),
                                                last_telemetry.sequence,
                                                last_telemetry.accel_x, last_telemetry.accel_y, last_telemetry.accel_z,
                                                last_telemetry.gyro_x, last_telemetry.gyro_y, last_telemetry.gyro_z,
                                                last_telemetry.pressure, last_telemetry.altitude, last_telemetry.temperature);
                    
                    telemetry.transmit(TelemetryChannel::SERIAL_DEBUG, telemetry_buffer);
                }
                
                last_telemetry_time = millis();
            }
            break;
            
        case MissionState::SHUTDOWN:
            // Finale Herunterfahren
            Serial.println("INFO: Mission complete");
            break;
    }
}

MissionState MissionControl::getCurrentState() const {
    return current_state;
}

uint32_t MissionControl::getMissionTime() const {
    if (mission_start_time == 0) return 0;
    return millis() - mission_start_time;
}

bool MissionControl::preflightCheck() {
    static uint32_t check_start = 0;
    
    if (check_start == 0) {
        check_start = millis();
        Serial.println("INFO: Starting preflight checks...");
    }
    
    Serial.println("Checking sensors...");
    if (data_collector.getSensorStatus() == 0) {
        Serial.println("ERROR: No sensors detected");
        return false;
    }
    
    Serial.println("Checking safety systems...");
    if (safety.checkPowerSupply() == false) {
        Serial.println("ERROR: Power supply check failed");
        return false;
    }
    
    Serial.println("Checking memory...");
    if (safety.checkMemory() == false) {
        Serial.println("ERROR: Memory check failed");
        return false;
    }
    
    Serial.println("All preflight checks passed!");
    check_start = 0;
    return true;
}

void MissionControl::startMission() {
    if (current_state != MissionState::STANDBY) {
        Serial.println("ERROR: Cannot start from current state");
        return;
    }
    
    mission_start_time = millis();
    experiment.calibrateSensors();
    experiment.start();
    onStateChange(MissionState::ASCENT);
}

void MissionControl::abortMission() {
    Serial.println("WARNING: Mission aborted!");
    experiment.reset();
    safety.emergencyShutdown();
}

void MissionControl::onStateChange(MissionState new_state) {
    if (current_state == new_state) return;
    
    current_state = new_state;
    printMissionStatus();
}

void MissionControl::printMissionStatus() {
    Serial.print("INFO: Mission state changed to: ");
    
    switch (current_state) {
        case MissionState::STARTUP:
            Serial.println("STARTUP");
            break;
        case MissionState::PREFLIGHT_CHECK:
            Serial.println("PREFLIGHT_CHECK");
            break;
        case MissionState::STANDBY:
            Serial.println("STANDBY");
            break;
        case MissionState::ASCENT:
            Serial.println("ASCENT");
            break;
        case MissionState::MICROGRAVITY:
            Serial.println("MICROGRAVITY");
            break;
        case MissionState::DESCENT:
            Serial.println("DESCENT");
            break;
        case MissionState::RECOVERY:
            Serial.println("RECOVERY");
            break;
        case MissionState::SHUTDOWN:
            Serial.println("SHUTDOWN");
            break;
    }
}
