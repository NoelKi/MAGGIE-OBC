#pragma once
#include <cstdint>
#include "hal/imu_hal.hpp"
#include "hal/telemetry_hal.hpp"
#include "hal/motor_hal.hpp"
#include "hal/force_hal.hpp"
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
 *   Kraftsensor 1       - 3x HX711 (X/Y/Z), Telemetrie in beiden Zustaenden
 *   Kraftsensor 2       - 4x HX711 (A/B/C/D), Verrechnung erst am Boden
 *   Down-/Uplink        - Serial4 (Pin 16/17) zum REXUS-Servicemodul
 *   REXUS-Signale       - werden eingelesen und als Rohpegel heruntergefunkt
 *
 * Weitere Komponenten (Kameras, Kraftsensor 2, Temperatur, Roboterarm) kommen
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
    TelemetryDownlink* downlink_ = nullptr;            ///< Downlink telemetry (Serial4, pins 16/17)
    MotorHAL* motor_ = nullptr;                         ///< Motor 1 (DRV8871, Open-Loop + Encoder als Sensor)
    ForceHAL* force1_ = nullptr;                        ///< Kraftsensor 1 (3x HX711, gemeinsamer Takt)
    ForceHAL* force2_ = nullptr;                        ///< Kraftsensor 2 (4x HX711, gemeinsamer Takt)
    UplinkReceiver* uplink_ = nullptr;                 ///< Telecommand-Empfang (Serial4 RX)
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
    bool force1_ready_ = false;             ///< Kraftsensor 1 initialisiert
    bool force2_ready_ = false;             ///< Kraftsensor 2 initialisiert

    // -----------------------------------------------------------------------
    // Kraftsensoren
    // -----------------------------------------------------------------------
    // Der HX711 wandelt mit 10 Hz. Gepollt wird trotzdem jeden Loop (read()
    // kehrt ohne neuen Messwert sofort zurueck) - so geht kein Sample verloren
    // und der Downlink laeuft automatisch im Takt des Sensors statt in einem
    // festen Intervall, das gegen die Wandlung schwebt.
    //
    // Jeder Sensor fuehrt seinen eigenen Zustand: Die beiden Gruppen haengen an
    // getrennten Taktleitungen und wandeln unabhaengig voneinander.
    struct ForceChannelState {
        ForceReading last{};                ///< letzter gelesener Messwert (0 vor dem ersten)
        uint32_t last_tx_ms = 0;            ///< Zeitstempel des letzten FORCE-Frames
    };
    ForceChannelState force1_state_;
    ForceChannelState force2_state_;

    /// Sendeintervall, solange ein Sensor haengt. Ohne diese Bremse ginge bei
    /// stehendem Wandler in JEDEM Loop ein Frame raus (siehe handleForceSensor).
    static constexpr uint32_t FORCE_STALE_TX_INTERVAL_MS = 1000;

    void printWelcomeBanner();
    void handleTelemetry();
    void handleMotorTelemetry();
    void handleForce();

    /// Einen Kraftsensor pollen und bei neuem Messwert (oder Haenger) senden.
    void handleForceSensor(ForceHAL* hal, bool ready, DownlinkForceMsg msg,
                           ForceChannelState& state);

    /// Einen Kraftsensor tarieren und das Ergebnis am Terminal protokollieren.
    void tareForceSensor(ForceHAL* hal, bool ready, const char* label);

    /// Pins und Nullpunkte eines Kraftsensors ans USB-Terminal melden.
    static void logForceSensor(ForceHAL* hal, const char* label,
                               const uint8_t* pins, uint8_t count, uint8_t sck);
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
