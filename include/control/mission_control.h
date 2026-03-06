#pragma once

#include <Arduino.h>
#include <cstdint>
#include "services/data_collection_service.h"
#include "services/telemetry_service.h"
#include "control/safety_monitor.h"
#include "control/flight_experiment_controller.h"
#include "control/ground_test_controller.h"
#include "control/preflight_test_controller.h"

/**
 * @brief Top-Level Mission States
 * 
 * Beschreibt den Betriebsmodus des OBC.
 * MissionControl verwaltet NUR den Lifecycle — keine Flugphasen.
 * Flugphasen werden vom FlightExperimentController erkannt und gesteuert.
 */
enum class MissionState {
    STARTUP,           // Boot-Phase: HAL + Services initialisieren
    PREFLIGHT_CHECK,   // Automatische System-Checks
    STANDBY,           // Bereit: Warte auf Modus-Auswahl (F/T/H/S)
    EXPERIMENT,        // Flug-Modus: FlightExperimentController steuert alles
    TESTING,           // Test-Modus: PreflightTestController validiert Sequenzen
    HARDWARE_TESTING,  // HW-Test-Modus: GroundTestController testet Einzelgeräte
    SHUTDOWN,          // System herunterfahren
};

/**
 * @brief Betriebsmodus — gewählt aus STANDBY
 */
enum class OperationMode {
    NONE,
    FLIGHT,
    TESTING,
    HARDWARE_TEST,
};

/**
 * @brief Zentrale Mission-Kontrolleinheit
 * 
 * Verantwortlich für:
 * - Boot-Sequenz & Preflight-Checks
 * - Modus-Auswahl (Flight / Testing / Hardware-Test)
 * - Delegation an den richtigen Controller
 * - Telemetrie & Safety-Überwachung
 * - Shutdown & Emergency
 * 
 * NICHT verantwortlich für:
 * - Flugphasen-Details (→ FlightExperimentController)
 * - Hardware-Test-Details (→ GroundTestController)
 * - Preflight-Test-Details (→ PreflightTestController)
 */
class MissionControl {
public:
    MissionControl();
    ~MissionControl();

    bool init();
    void run();

    // State
    MissionState getCurrentState() const;
    OperationMode getOperationMode() const;
    uint32_t getMissionTime() const;

    // Modus-Auswahl (nur aus STANDBY)
    bool selectMode(OperationMode mode);

    // Shutdown & Emergency
    void requestShutdown();
    void abortMission();

private:
    MissionState current_state = MissionState::STARTUP;
    OperationMode operation_mode = OperationMode::NONE;
    uint32_t boot_time = 0;
    uint32_t state_entry_time = 0;
    uint32_t last_telemetry_time = 0;
    uint8_t preflight_retry_count = 0;

    // Controller — je nach Modus aktiv
    FlightExperimentController flight_controller;
    GroundTestController ground_test_controller;
    PreflightTestController preflight_test_controller;

    // Services — immer aktiv
    SafetyMonitor safety;
    DataCollectionService data_collector;
    TelemetryService telemetry;

    // Telemetrie-Buffer
    TelemetryData last_telemetry = {};
    char telemetry_buffer[256];

    static constexpr uint32_t TELEMETRY_INTERVAL = 100;  // 10 Hz
    static constexpr uint32_t BOOT_DELAY = 3000;          // 3s Boot
    static constexpr uint8_t  MAX_PREFLIGHT_RETRIES = 3;

    // State-Handler
    void handleStartup();
    void handlePreflightCheck();
    void handleStandby();
    void handleExperiment();
    void handleTesting();
    void handleHardwareTesting();
    void handleShutdown();

    // Preflight
    bool runPreflightChecks();

    // Telemetrie (in allen aktiven Modi)
    void collectAndSendTelemetry();

    // Kommando-Parsing (Serial)
    void checkForCommands();

    // State-Transition
    void transitionTo(MissionState new_state);
    const char* stateToString(MissionState state) const;
};
