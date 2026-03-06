#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Preflight Test Controller
 * Zum Testen kompletter Experiment-Sequenzen VOR dem Start
 * 
 * Funktionen:
 * - Teste Experiment-Ablauf (zeitlich komprimiert)
 * - Validiere Sensor-Sequenzen
 * - Teste Geräte-Koordination
 * - Simuliere Flugphasen
 * - Überprüfe Sicherheits-Mechanismen
 */

enum class PreflightTestState {
    IDLE,                    // Bereit
    RUNNING,                 // Test läuft
    SENSOR_VALIDATION,       // Validiere Sensoren
    SEQUENCE_TEST,           // Teste Experiment-Sequenzen
    EMERGENCY_STOP_TEST,     // Teste Notfall-Abschaltung
    TIMING_VALIDATION,       // Validiere Timing
    COMPLETE,                // Test abgeschlossen
    FAILED,                  // Test fehlgeschlagen
};

class PreflightTestController {
public:
    PreflightTestController();
    ~PreflightTestController();
    
    /**
     * @brief Initialisiert Preflight Test Controller
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Update-Loop für Preflight Tests
     */
    void update();
    
    /**
     * @brief Starte kompletten Preflight-Test
     * @param compressed_time true = schneller Test (1 Sekunde = 1 Flug-Minute)
     */
    void runFullTest(bool compressed_time = true);
    
    /**
     * @brief Teste nur Sensor-Validierung
     */
    void testSensorValidation();
    
    /**
     * @brief Teste Experiment-Sequenz
     * @param sequence_id Sequenz-ID (0=Start, 1=Ascent, 2=Microgravity, etc.)
     */
    void testExperimentSequence(uint8_t sequence_id);
    
    /**
     * @brief Teste Emergency-Stop-Funktion
     */
    void testEmergencyStop();
    
    /**
     * @brief Teste Timing-Genauigkeit
     */
    void testTimingValidation();
    
    /**
     * @brief Simuliere Flugphase (zeitlich komprimiert)
     * @param phase_id 0=ASCENT, 1=MICROGRAVITY, 2=DESCENT
     * @param duration_ms Simulationsdauer in ms
     */
    void simulateFlightPhase(uint8_t phase_id, uint32_t duration_ms);
    
    /**
     * @brief Gibt aktuellen Test-Status zurück
     */
    PreflightTestState getStatus() const;
    
    /**
     * @brief Gibt Fehler-Nachricht zurück (wenn Test fehlgeschlagen)
     */
    const char* getErrorMessage() const;
    
    /**
     * @brief Stoppe aktuellen Test
     */
    void stopTest();
    
    /**
     * @brief Printe Test-Report
     */
    void printTestReport();

private:
    PreflightTestState current_state = PreflightTestState::IDLE;
    uint32_t test_start_time = 0;
    uint32_t test_elapsed_time = 0;
    bool use_compressed_time = true;
    char error_message[128] = "";
    uint32_t passed_tests = 0;
    uint32_t failed_tests = 0;
    
    bool validateSensors();
    bool validateExperimentSequence(uint8_t sequence_id);
    bool validateEmergencyStop();
    bool validateTiming();
    void recordTestResult(const char* test_name, bool passed);
};
