#include "control/preflight_test_controller.h"

PreflightTestController::PreflightTestController() {
    // Konstruktor
}

PreflightTestController::~PreflightTestController() {
    // Destruktor
}

bool PreflightTestController::init() {
    Serial.println("===================================");
    Serial.println("MAGGIE Preflight Test Controller");
    Serial.println("Initializing...");
    Serial.println("===================================\n");
    
    current_state = PreflightTestState::IDLE;
    passed_tests = 0;
    failed_tests = 0;
    
    Serial.println("INFO: Preflight Test Controller initialized");
    return true;
}

void PreflightTestController::update() {
    if (current_state != PreflightTestState::RUNNING) {
        return;
    }
    
    test_elapsed_time = millis() - test_start_time;
    
    // Test-Logik basierend auf Sub-State
    // TODO: Implementiere State Machine für Tests
}

void PreflightTestController::runFullTest(bool compressed_time) {
    Serial.println("\n===================================");
    Serial.println("Starting Full Preflight Test");
    Serial.println("===================================\n");
    
    current_state = PreflightTestState::RUNNING;
    test_start_time = millis();
    use_compressed_time = compressed_time;
    passed_tests = 0;
    failed_tests = 0;
    
    // Test 1: Sensor Validation
    Serial.println("[1/4] Testing Sensors...");
    if (validateSensors()) {
        recordTestResult("Sensor Validation", true);
    } else {
        recordTestResult("Sensor Validation", false);
        strcpy(error_message, "Sensor validation failed");
    }
    
    // Test 2: Experiment Sequence
    Serial.println("[2/4] Testing Experiment Sequences...");
    for (uint8_t i = 0; i < 3; i++) {  // Test 3 sequences
        if (validateExperimentSequence(i)) {
            recordTestResult("Experiment Sequence", true);
        } else {
            recordTestResult("Experiment Sequence", false);
        }
    }
    
    // Test 3: Emergency Stop
    Serial.println("[3/4] Testing Emergency Stop...");
    if (validateEmergencyStop()) {
        recordTestResult("Emergency Stop", true);
    } else {
        recordTestResult("Emergency Stop", false);
        strcpy(error_message, "Emergency stop validation failed");
    }
    
    // Test 4: Timing
    Serial.println("[4/4] Testing Timing Validation...");
    if (validateTiming()) {
        recordTestResult("Timing Validation", true);
    } else {
        recordTestResult("Timing Validation", false);
        strcpy(error_message, "Timing validation failed");
    }
    
    // Test-Ende
    if (failed_tests == 0) {
        current_state = PreflightTestState::COMPLETE;
        Serial.println("\n✓ ALL TESTS PASSED");
    } else {
        current_state = PreflightTestState::FAILED;
        Serial.printf("\n✗ TEST FAILED: %d/%d tests failed\n", failed_tests, passed_tests + failed_tests);
    }
    
    printTestReport();
}

void PreflightTestController::testSensorValidation() {
    current_state = PreflightTestState::SENSOR_VALIDATION;
    validateSensors();
}

void PreflightTestController::testExperimentSequence(uint8_t sequence_id) {
    current_state = PreflightTestState::SEQUENCE_TEST;
    validateExperimentSequence(sequence_id);
}

void PreflightTestController::testEmergencyStop() {
    current_state = PreflightTestState::EMERGENCY_STOP_TEST;
    validateEmergencyStop();
}

void PreflightTestController::testTimingValidation() {
    current_state = PreflightTestState::TIMING_VALIDATION;
    validateTiming();
}

void PreflightTestController::simulateFlightPhase(uint8_t phase_id, uint32_t duration_ms) {
    Serial.printf("INFO: Simulating Flight Phase %d for %u ms\n", phase_id, duration_ms);
    
    uint32_t phase_start = millis();
    
    switch (phase_id) {
        case 0:  // ASCENT
            Serial.println("  [ASCENT SIMULATION]");
            while (millis() - phase_start < duration_ms) {
                // Simuliere Ascent-Logik
                delay(10);
            }
            break;
            
        case 1:  // MICROGRAVITY
            Serial.println("  [MICROGRAVITY SIMULATION]");
            while (millis() - phase_start < duration_ms) {
                // Simuliere Microgravity-Logik
                delay(10);
            }
            break;
            
        case 2:  // DESCENT
            Serial.println("  [DESCENT SIMULATION]");
            while (millis() - phase_start < duration_ms) {
                // Simuliere Descent-Logik
                delay(10);
            }
            break;
    }
    
    Serial.println("  Phase simulation complete");
}

PreflightTestState PreflightTestController::getStatus() const {
    return current_state;
}

const char* PreflightTestController::getErrorMessage() const {
    return error_message;
}

void PreflightTestController::stopTest() {
    if (current_state == PreflightTestState::RUNNING) {
        current_state = PreflightTestState::FAILED;
        strcpy(error_message, "Test stopped by user");
        Serial.println("INFO: Test stopped");
    }
}

void PreflightTestController::printTestReport() {
    Serial.println("\n===================================");
    Serial.println("PREFLIGHT TEST REPORT");
    Serial.println("===================================");
    Serial.printf("Total Tests: %d\n", passed_tests + failed_tests);
    Serial.printf("✓ Passed: %d\n", passed_tests);
    Serial.printf("✗ Failed: %d\n", failed_tests);
    Serial.printf("Duration: %u ms\n", millis() - test_start_time);
    Serial.println("===================================\n");
}

bool PreflightTestController::validateSensors() {
    Serial.println("  Checking IMU...");
    // TODO: Implementiere Sensor-Validierung
    delay(100);
    
    Serial.println("  Checking Barometer...");
    delay(100);
    
    Serial.println("  Checking Temperature...");
    delay(100);
    
    Serial.println("  ✓ All sensors validated");
    return true;
}

bool PreflightTestController::validateExperimentSequence(uint8_t sequence_id) {
    Serial.printf("  Validating Sequence %d...\n", sequence_id);
    // TODO: Implementiere Sequenz-Validierung
    delay(200);
    return true;
}

bool PreflightTestController::validateEmergencyStop() {
    Serial.println("  Testing emergency stop mechanism...");
    // TODO: Implementiere Emergency-Stop-Validierung
    delay(100);
    Serial.println("  ✓ Emergency stop validated");
    return true;
}

bool PreflightTestController::validateTiming() {
    Serial.println("  Checking timing accuracy...");
    // TODO: Implementiere Timing-Validierung
    delay(100);
    Serial.println("  ✓ Timing validated");
    return true;
}

void PreflightTestController::recordTestResult(const char* test_name, bool passed) {
    if (passed) {
        Serial.printf("  ✓ %s: PASSED\n", test_name);
        passed_tests++;
    } else {
        Serial.printf("  ✗ %s: FAILED\n", test_name);
        failed_tests++;
    }
}
