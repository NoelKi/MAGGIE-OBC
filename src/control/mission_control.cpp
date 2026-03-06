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
        Serial.println("WARNING: Flight experiment controller initialization failed");
    }
    
    onStateChange(MissionState::STARTUP);
    Serial.println("INFO: Mission Control initialized successfully");
    
    return true;
}

void MissionControl::run() {
    // Update Safety immer
    safety.update();
    
    // Notfall-Check: Safety hat höchste Priorität
    if (safety.getCurrentLevel() == SafetyLevel::EMERGENCY) {
        experiment.emergencyStop("Safety monitor triggered emergency");
        abortMission();
        return;
    }
    
    // Handle current state
    switch (current_state) {
        case MissionState::STARTUP:
            // Auto-Transition nach Boot
            if (millis() > 5000) {
                onStateChange(MissionState::PREFLIGHT_CHECK);
            }
            break;
            
        case MissionState::PREFLIGHT_CHECK:
            // Automatische Preflight-Checks
            if (preflightCheck()) {
                // Arme das Experiment nach erfolgreichen Checks
                experiment.armExperiment();
                onStateChange(MissionState::STANDBY);
            }
            break;
            
        case MissionState::STANDBY:
            // Warte auf Lift-Off Signal (z.B. via CAN, Serial, oder Accelerometer)
            if (isLiftOffSignal()) {
                startMission();
            }
            break;
            
        case MissionState::ASCENT:
            // Aufstiegsphase: Experiment pausiert, Daten werden gesammelt
            experiment.update();
            
            // Erkennung: Schwerelosigkeit (Apogee erreicht)
            if (detectApogee()) {
                experiment.onFlightPhaseChange(FlightPhase::MICROGRAVITY);
                onStateChange(MissionState::MICROGRAVITY);
            }
            break;
            
        case MissionState::MICROGRAVITY:
            // Schwerelosigkeitsphase: Experiment läuft aktiv!
            experiment.update();
            
            // Erkennung: Abstieg beginnt
            if (detectDescent()) {
                experiment.onFlightPhaseChange(FlightPhase::DESCENT);
                onStateChange(MissionState::DESCENT);
            }
            break;
            
        case MissionState::DESCENT:
            // Abstiegsphase: Experiment pausiert
            experiment.update();
            
            // Erkennung: Landung
            if (detectLanding()) {
                experiment.onFlightPhaseChange(FlightPhase::RECOVERY);
                onStateChange(MissionState::RECOVERY);
            }
            break;
            
        case MissionState::RECOVERY:
            // Bergungsphase: Daten sichern
            experiment.update();
            
            // Nach 30 Sekunden Recovery → Shutdown
            if (getMissionTime() > 0 && (millis() - mission_start_time) > 300000) {
                onStateChange(MissionState::SHUTDOWN);
            }
            break;
            
        case MissionState::SHUTDOWN:
            Serial.println("INFO: Mission complete - shutting down");
            break;
    }
    
    // Telemetrie in allen aktiven Flugphasen senden
    if (current_state >= MissionState::ASCENT && current_state <= MissionState::RECOVERY) {
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

bool MissionControl::isLiftOffSignal() {
    // TODO: Implementiere echte Start-Signal-Erkennung
    // Optionen:
    // 1. CAN-Bus Kommando vom Service-Modul
    // 2. Accelerometer-Schwelle (z.B. > 3g)
    // 3. Serial-Kommando (Debug)
    return false;
}

void MissionControl::startMission() {
    if (current_state != MissionState::STANDBY) {
        Serial.println("ERROR: Cannot start from current state");
        return;
    }
    
    mission_start_time = millis();
    experiment.onFlightPhaseChange(FlightPhase::ASCENT);
    onStateChange(MissionState::ASCENT);
    
    Serial.println("INFO: Mission started - ASCENT phase");
}

void MissionControl::abortMission() {
    Serial.println("WARNING: Mission aborted!");
    experiment.emergencyStop("Mission aborted");
    safety.emergencyShutdown();
    onStateChange(MissionState::SHUTDOWN);
}

bool MissionControl::detectApogee() {
    // Erkennung der Schwerelosigkeit:
    // Beschleunigung nahe 0g (alle Achsen < 0.5 m/s²)
    float accel_mag = sqrtf(last_telemetry.accel_x * last_telemetry.accel_x +
                           last_telemetry.accel_y * last_telemetry.accel_y +
                           last_telemetry.accel_z * last_telemetry.accel_z);
    
    // Schwerelosigkeit: Beschleunigung < 0.5 m/s² (nahe 0g)
    return accel_mag < 0.5f;
}

bool MissionControl::detectDescent() {
    // Erkennung des Abstiegs:
    // Beschleunigung steigt wieder > 1g UND Höhe sinkt
    float accel_mag = sqrtf(last_telemetry.accel_x * last_telemetry.accel_x +
                           last_telemetry.accel_y * last_telemetry.accel_y +
                           last_telemetry.accel_z * last_telemetry.accel_z);
    
    // Wenn Beschleunigung wieder deutlich über 0g steigt → Abstieg beginnt
    static float last_altitude = 0.0f;
    bool altitude_decreasing = (last_telemetry.altitude < last_altitude - 10.0f);
    last_altitude = last_telemetry.altitude;
    
    return (accel_mag > 5.0f) && altitude_decreasing;
}

bool MissionControl::detectLanding() {
    // Erkennung der Landung:
    // Beschleunigung ca. 1g UND Höhe stabil (nahe 0)
    float accel_mag = sqrtf(last_telemetry.accel_x * last_telemetry.accel_x +
                           last_telemetry.accel_y * last_telemetry.accel_y +
                           last_telemetry.accel_z * last_telemetry.accel_z);
    
    // ~1g (9.81 ± 1.0 m/s²) UND Altitude < 500m
    bool gravity_normal = (accel_mag > 8.8f && accel_mag < 10.8f);
    bool low_altitude = (last_telemetry.altitude < 500.0f);
    
    return gravity_normal && low_altitude;
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
