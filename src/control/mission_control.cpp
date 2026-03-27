#include "control/mission_control.h"
#include <cmath>

MissionControl::MissionControl() {}
MissionControl::~MissionControl() {}

// ============================================================================
// Initialisierung
// ============================================================================

bool MissionControl::init() {
    Serial.println("===================================");
    Serial.println("  MAGGIE OBC — Mission Control");
    Serial.println("  REXUS Programme v1.0");
    Serial.println("===================================\n");

    boot_time = millis();

    // Safety zuerst — kritisch
    if (!safety.init()) {
        Serial.println("FATAL: SafetyMonitor init failed");
        return false;
    }

    // Datenerfassung
    if (!data_collector.init()) {
        Serial.println("WARN: DataCollectionService init failed");
    }

    // Telemetrie
    if (!telemetry.init()) {
        Serial.println("WARN: TelemetryService init failed");
    }

    transitionTo(MissionState::STARTUP);
    Serial.println("INFO: MissionControl initialized\n");
    return true;
}

// ============================================================================
// Haupt-Loop
// ============================================================================

void MissionControl::run() {
    // 1) Safety hat immer höchste Priorität
    // return;
    
    safety.update();

    if (safety.getCurrentLevel() == SafetyLevel::EMERGENCY) {
        Serial.println("EMERGENCY: Safety triggered abort!");
        abortMission();
        return;
    }

    // 2) State Machine
    switch (current_state) {
        case MissionState::STARTUP:          handleStartup();          break;
        case MissionState::PREFLIGHT_CHECK:  handlePreflightCheck();   break;
        case MissionState::STANDBY:          handleStandby();          break;
        case MissionState::EXPERIMENT:       handleExperiment();       break;
        case MissionState::TESTING:          handleTesting();          break;
        case MissionState::HARDWARE_TESTING: handleHardwareTesting();  break;
        case MissionState::SHUTDOWN:         handleShutdown();         break;
    }
}

// ============================================================================
// State Handlers
// ============================================================================

void MissionControl::handleStartup() {
    Serial.printf("Mission Control ist am Controllen\n");
    if (millis() - state_entry_time > BOOT_DELAY) {
        transitionTo(MissionState::PREFLIGHT_CHECK);
    }
}

void MissionControl::handlePreflightCheck() {
    if (runPreflightChecks()) {
        Serial.println("INFO: All preflight checks PASSED\n");
        preflight_retry_count = 0;
        transitionTo(MissionState::STANDBY);
    } else {
        preflight_retry_count++;
        Serial.printf("WARN: Preflight failed (attempt %d/%d)\n",
                      preflight_retry_count, MAX_PREFLIGHT_RETRIES);

        if (preflight_retry_count >= MAX_PREFLIGHT_RETRIES) {
            Serial.println("ERROR: Max preflight retries → SHUTDOWN");
            transitionTo(MissionState::SHUTDOWN);
        }
    }
}

void MissionControl::handleStandby() {
    // Warte auf Modus-Auswahl via Serial oder CAN
    checkForCommands();

    // Info alle 10 Sekunden
    static uint32_t last_info = 0;
    if (millis() - last_info > 10000) {
        Serial.println("STANDBY: Awaiting mode selection");
        Serial.println("  [F] Flight  [T] Testing  [H] Hardware Test  [S] Shutdown");
        last_info = millis();
    }
}

void MissionControl::handleExperiment() {
    // Sensordaten sammeln und an FlightController weitergeben
    if (data_collector.collectData(last_telemetry)) {
        flight_controller.feedSensorData(
            last_telemetry.accel_x, last_telemetry.accel_y, last_telemetry.accel_z,
            last_telemetry.pressure, last_telemetry.altitude
        );

        // Safety prüfen
        float accel_mag = sqrtf(
            last_telemetry.accel_x * last_telemetry.accel_x +
            last_telemetry.accel_y * last_telemetry.accel_y +
            last_telemetry.accel_z * last_telemetry.accel_z);
        safety.checkSensorBounds(accel_mag, last_telemetry.temperature, last_telemetry.altitude);
    }

    // FlightController erkennt Phasen und steuert Experiment
    flight_controller.update();

    // Telemetrie senden
    collectAndSendTelemetry();

    // Experiment beendet?
    if (flight_controller.getState() == FlightExperimentState::FINISHED) {
        Serial.println("INFO: Experiment finished → SHUTDOWN");
        transitionTo(MissionState::SHUTDOWN);
    }

    // Fehler im Experiment?
    if (flight_controller.hasError()) {
        Serial.printf("ERROR: Experiment error 0x%04X → SHUTDOWN\n",
                      flight_controller.getErrorCode());
        transitionTo(MissionState::SHUTDOWN);
    }
}

void MissionControl::handleTesting() {
    preflight_test_controller.update();

    PreflightTestState test_status = preflight_test_controller.getStatus();
    if (test_status == PreflightTestState::COMPLETE ||
        test_status == PreflightTestState::FAILED) {
        preflight_test_controller.printTestReport();
        transitionTo(MissionState::SHUTDOWN);
    }
}

void MissionControl::handleHardwareTesting() {
    //ground_test_controller.update();

    // Serial-Kommandos an GroundTestController weiterleiten
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.equalsIgnoreCase("EXIT") || cmd.equalsIgnoreCase("QUIT")) {
            transitionTo(MissionState::SHUTDOWN);
        } else {
            ground_test_controller.handleSerialCommand(cmd);
        }
    }
}

void MissionControl::handleShutdown() {
    static bool done = false;
    if (done) return;

    Serial.println("\n===================================");
    Serial.println("  SHUTDOWN SEQUENCE");
    Serial.println("===================================");

    // Experiment sicher stoppen
    if (operation_mode == OperationMode::FLIGHT) {
        flight_controller.emergencyStop("Shutdown sequence");
    }

    // Safety
    safety.emergencyShutdown();

    Serial.printf("INFO: Total uptime: %lu ms\n", millis() - boot_time);
    Serial.println("INFO: Shutdown complete. Safe to power off.\n");
    done = true;
}

// ============================================================================
// Modus-Auswahl
// ============================================================================

bool MissionControl::selectMode(OperationMode mode) {
    if (current_state != MissionState::STANDBY) {
        Serial.println("ERROR: Mode selection only available in STANDBY");
        return false;
    }

    operation_mode = mode;

    switch (mode) {
        case OperationMode::FLIGHT:
            Serial.println("\nMODE: FLIGHT selected");
            flight_controller.init();
            flight_controller.armExperiment();
            transitionTo(MissionState::EXPERIMENT);
            break;

        case OperationMode::TESTING:
            Serial.println("\nMODE: TESTING selected");
            preflight_test_controller.init();
            preflight_test_controller.runFullTest(true);
            transitionTo(MissionState::TESTING);
            break;

        case OperationMode::HARDWARE_TEST:
            Serial.println("\nMODE: HARDWARE TEST selected");
            ground_test_controller.init();
            transitionTo(MissionState::HARDWARE_TESTING);
            break;

        default:
            return false;
    }
    return true;
}

// ============================================================================
// Preflight Checks
// ============================================================================

bool MissionControl::runPreflightChecks() {
    Serial.println("INFO: Running preflight checks...");
    bool ok = true;

    // 1) Sensoren
    // if (data_collector.getSensorStatus() == 0) {
    //     Serial.println("  ✗ Sensors: NONE detected");
    //     ok = false;
    // } else {
    //     Serial.println("  ✓ Sensors: OK");
    // }

    // 2) Stromversorgung
    if (!safety.checkPowerSupply()) {
        Serial.println("  ✗ Power: FAILED");
        ok = false;
    } else {
        Serial.println("  ✓ Power: OK");
    }

    // 3) Speicher
    if (!safety.checkMemory()) {
        Serial.println("  ✗ Memory: LOW");
        ok = false;
    } else {
        Serial.println("  ✓ Memory: OK");
    }

    return ok;
}

// ============================================================================
// Telemetrie
// ============================================================================

void MissionControl::collectAndSendTelemetry() {
    if (millis() - last_telemetry_time < TELEMETRY_INTERVAL) return;

    if (data_collector.collectData(last_telemetry)) {
        telemetry.formatTelemetryCSV(
            telemetry_buffer, sizeof(telemetry_buffer),
            last_telemetry.sequence,
            last_telemetry.accel_x, last_telemetry.accel_y, last_telemetry.accel_z,
            last_telemetry.gyro_x, last_telemetry.gyro_y, last_telemetry.gyro_z,
            last_telemetry.pressure, last_telemetry.altitude, last_telemetry.temperature);

        telemetry.transmit(TelemetryChannel::SERIAL_DEBUG, telemetry_buffer);
    }

    last_telemetry_time = millis();
}

// ============================================================================
// Serial-Kommandos
// ============================================================================

void MissionControl::checkForCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    switch (cmd) {
        case 'F': case 'f': selectMode(OperationMode::FLIGHT);        break;
        case 'T': case 't': selectMode(OperationMode::TESTING);       break;
        case 'H': case 'h': selectMode(OperationMode::HARDWARE_TEST); break;
        case 'S': case 's': requestShutdown();                        break;
        default:
            Serial.printf("Unknown command: '%c'\n", cmd);
            break;
    }
}

// ============================================================================
// State Transitions & Getters
// ============================================================================

void MissionControl::transitionTo(MissionState new_state) {
    if (current_state == new_state) return;

    Serial.printf("STATE: %s → %s\n",
                  stateToString(current_state),
                  stateToString(new_state));

    current_state = new_state;
    state_entry_time = millis();
}

MissionState MissionControl::getCurrentState() const {
    return current_state;
}

OperationMode MissionControl::getOperationMode() const {
    return operation_mode;
}

uint32_t MissionControl::getMissionTime() const {
    return millis() - boot_time;
}

void MissionControl::requestShutdown() {
    Serial.println("INFO: Shutdown requested");
    transitionTo(MissionState::SHUTDOWN);
}

void MissionControl::abortMission() {
    Serial.println("EMERGENCY: Mission abort!");
    if (operation_mode == OperationMode::FLIGHT) {
        flight_controller.emergencyStop("Mission abort");
    }
    safety.emergencyShutdown();
    transitionTo(MissionState::SHUTDOWN);
}

const char* MissionControl::stateToString(MissionState state) const {
    switch (state) {
        case MissionState::STARTUP:          return "STARTUP";
        case MissionState::PREFLIGHT_CHECK:  return "PREFLIGHT_CHECK";
        case MissionState::STANDBY:          return "STANDBY";
        case MissionState::EXPERIMENT:       return "EXPERIMENT";
        case MissionState::TESTING:          return "TESTING";
        case MissionState::HARDWARE_TESTING: return "HARDWARE_TESTING";
        case MissionState::SHUTDOWN:         return "SHUTDOWN";
        default:                             return "UNKNOWN";
    }
}
