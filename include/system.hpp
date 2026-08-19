#pragma once
#include <cstdint>
#include "drivers/weight_sensor_driver.hpp"
#include "hal/force_sensor_hal.hpp"
#include "hal/sensor_hal.hpp"
#include "hal/imu_hal.hpp"
#include "hal/telemetry_hal.hpp"
#include "hal/motor_hal.hpp"
#include "hal/uplink_hal.hpp"
#include "hal/rexus_hal.hpp"
#include "statemachine/state_machine.hpp"
#include "camera_config.hpp"
#include "pin_config.hpp"

/**
 * @brief Zentrale System-Klasse
 * Verwaltet den Gesamtzustand des On-Board Computers
 */

class System {
public:
    System();
    ~System();
    
    /**
     * @brief Initialisiert das gesamte System
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Hauptkontroll-Loop
     * Sollte kontinuierlich in loop() aufgerufen werden
     */
    void run();
    

private:
    uint32_t startup_time = 0;
    bool system_healthy = false;

    // -----------------------------------------------------------------------
    // Subsysteme
    // -----------------------------------------------------------------------
    WeightSensorDriver* weight_sensor_ = nullptr;      ///< HX711 Weight Sensor (Sensor 1)
    ForceSensorHAL* force_sensor_2_ = nullptr;         ///< Custom Force Sensor (Sensor 2)
    IMUHAL* imu_ = nullptr;                             ///< BMI088 IMU (SPI)
    TelemetryDownlink* downlink_ = nullptr;            ///< Downlink telemetry (Serial8, pins 34/35)
    MotorHAL* motor_ = nullptr;                         ///< Motor 1 (DRV8871 + Encoder, Closed-Loop)
    UplinkReceiver* uplink_ = nullptr;                 ///< Telecommand-Empfang (Serial8 RX)
    REXUSHAL* rexus_ = nullptr;                         ///< REXUS-Signale L0/SOE/SODS
    CameraBus* camera_bus_ = nullptr;                   ///< gemeinsamer UART + Mux für alle Kameras
    CameraHAL* cameras_[CAMERA_COUNT] = {};             ///< siehe camera_config.hpp für die Liste
    bool camera_reported_[CAMERA_COUNT] = {};           ///< Handshake-Ergebnis bereits geloggt?

    // -----------------------------------------------------------------------
    // Missions-Zustandsmaschine
    // -----------------------------------------------------------------------
    StateMachine state_machine_;                        ///< PRE_LAUNCH/TEST/Flugsequenz
    MissionState last_state_ = MissionState::PRE_LAUNCH; ///< für die Zustandswechsel-Erkennung
    bool abort_requested_ = false;                       ///< gelatchter Abbruch per Telecommand

    uint32_t last_weight_read_ms_ = 0;      ///< Zeitstempel des letzten Weight-Auslesens
    uint32_t last_force_read_ms_ = 0;       ///< Zeitstempel des letzten Force-Auslesens
    uint32_t last_telemetry_ms_ = 0;        ///< Zeitstempel des letzten Telemetrie-Downlinks
    uint32_t last_sys_telemetry_ms_ = 0;    ///< Zeitstempel des letzten SYS/STATE-Downlinks

    // Kein Subsystem-Fehler ist fatal: der OBC laeuft degradiert weiter, statt
    // z.B. wegen eines fehlenden Wiegesensors Telemetrie und Kameras mit
    // abzuschalten. system_healthy fasst zusammen, ob ALLE Subsysteme oben
    // sind - das Bit geht per DL_STATUS1_SYSTEM_HEALTHY an die Bodenstation.
    bool weight_sensor_ready_ = false;      ///< HX711 erfolgreich initialisiert
    bool force_sensor_ready_ = false;       ///< Kraftsensor 2 erfolgreich initialisiert
    bool imu_ready_ = false;                ///< IMU erfolgreich initialisiert
    bool downlink_ready_ = false;           ///< Downlink-UART bereit
    bool motor_ready_ = false;              ///< Motor (inkl. Encoder) initialisiert
    bool rexus_ready_ = false;              ///< REXUS-Signalpins konfiguriert

    void printWelcomeBanner();
    void handleWeightReading();
    void handleForceReading();
    void handleTelemetry();
    void handleMotorTelemetry();
    void handleSystemTelemetry(uint32_t now_ms);
    void handleStateMachine(uint32_t now_ms);
    void onStateChanged(MissionState previous, MissionState current);
    void handleUplink(uint32_t now_ms);
    bool handleMotorCommand(const UplinkCommand& cmd);
    uint8_t subsystemBits() const;
    void handleCameras(uint32_t now_ms);
};

// Externe System-Instanz (definiert in main.cpp)
extern System* g_system;