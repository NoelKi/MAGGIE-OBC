#pragma once
#include <cstdint>
#include "hal/imu_hal.hpp"
#include "hal/telemetry_hal.hpp"
#include "hal/motor_hal.hpp"
#include "hal/uplink_hal.hpp"
#include "hal/rexus_hal.hpp"
#include "statemachine/state_machine.hpp"
#include "pin_config.hpp"

/**
 * @brief Zentrale System-Klasse
 *
 * Verwaltet den Gesamtzustand des On-Board Computers. Der aktuelle Ausbau ist
 * bewusst schlank gehalten und umfasst genau die Subsysteme, die auf dem
 * Testaufbau verbaut sind:
 *
 *   IMU (BMI088)        - Telemetrie in beiden Zustaenden
 *   Motor 1 + Encoder   - HDRM-Antrieb, per Telecommand nur im TEST-Zustand
 *   Down-/Uplink        - Serial8 (Pin 34/35) zum REXUS-Servicemodul
 *   REXUS-Signale       - werden eingelesen und als Rohpegel heruntergefunkt
 *
 * Weitere Komponenten (Kameras, Wiegezelle, Kraftsensoren, Roboterarm) kommen
 * Stueck fuer Stueck dazu, sobald sie am Aufbau haengen und getestet werden.
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
    IMUHAL* imu_ = nullptr;                             ///< BMI088 IMU (SPI)
    TelemetryDownlink* downlink_ = nullptr;            ///< Downlink telemetry (Serial8, pins 34/35)
    MotorHAL* motor_ = nullptr;                         ///< Motor 1 (DRV8871, Open-Loop + Encoder als Sensor)
    UplinkReceiver* uplink_ = nullptr;                 ///< Telecommand-Empfang (Serial8 RX)
    REXUSHAL* rexus_ = nullptr;                         ///< REXUS-Signale L0/SOE/SODS

    // -----------------------------------------------------------------------
    // Zustandsmaschine
    // -----------------------------------------------------------------------
    StateMachine state_machine_;                        ///< PRE_LAUNCH/TEST/ABORT
    MissionState last_state_ = MissionState::PRE_LAUNCH; ///< für die Zustandswechsel-Erkennung
    bool abort_requested_ = false;                       ///< gelatchter Abbruch per Telecommand

    uint32_t last_telemetry_ms_ = 0;        ///< Zeitstempel des letzten Telemetrie-Downlinks
    uint32_t last_sys_telemetry_ms_ = 0;    ///< Zeitstempel des letzten SYS/STATE-Downlinks

    // -----------------------------------------------------------------------
    // Laufzeitbegrenzung des Motors
    // -----------------------------------------------------------------------
    // Der Motor laeuft Open-Loop und stoppt nicht mehr von selbst. Diese Grenze
    // ist die Rueckfallebene, falls MOTOR_OFF nicht durchkommt. 0 = aus.
    static constexpr uint32_t MOTOR_ON_TIMEOUT_MS = 30000;
    uint32_t motor_on_since_ms_ = 0;        ///< Zeitpunkt des letzten MOTOR_ON

    // Kein Subsystem-Fehler ist fatal: der OBC laeuft degradiert weiter, statt
    // z.B. wegen eines fehlenden Motors auch die Telemetrie abzuschalten.
    // system_healthy fasst zusammen, ob ALLE Subsysteme oben sind - das Bit
    // geht per DL_STATUS1_SYSTEM_HEALTHY an die Bodenstation.
    bool imu_ready_ = false;                ///< IMU erfolgreich initialisiert
    bool downlink_ready_ = false;           ///< Downlink-UART bereit
    bool motor_ready_ = false;              ///< Motor (inkl. Encoder) initialisiert
    bool rexus_ready_ = false;              ///< REXUS-Signalpins konfiguriert

    void printWelcomeBanner();
    void handleTelemetry();
    void handleMotorTelemetry();
    void handleSystemTelemetry(uint32_t now_ms);
    void handleStateMachine(uint32_t now_ms);
    void onStateChanged(MissionState previous, MissionState current);
    void handleUplink(uint32_t now_ms);
    bool handleMotorCommand(const UplinkCommand& cmd);
    void handleMotorTimeout(uint32_t now_ms);
    uint8_t subsystemBits() const;
};

// Externe System-Instanz (definiert in main.cpp)
extern System* g_system;
