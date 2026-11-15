#pragma once
#include <cstdint>
#include "hal/imu_hal.hpp"
#include "hal/telemetry_hal.hpp"
#include "hal/motor_hal.hpp"
#include "hal/force_hal.hpp"
#include "hal/uplink_hal.hpp"
#include "hal/rexus_hal.hpp"
#include "hal/arm_hal.hpp"
#include "arm_traj_player.hpp"
#include "statemachine/state_machine.hpp"
#include "storage/persistent_state.hpp"
#include "storage/sd_logger.hpp"
#include "pin_config.hpp"
#include "mission_timeline.hpp"
#include "data_rates.hpp"

/**
 * @brief Zentrale System-Klasse
 *
 * Verwaltet den Gesamtzustand des On-Board Computers. Der aktuelle Ausbau ist
 * bewusst schlank gehalten und umfasst genau die Subsysteme, die auf dem
 * Testaufbau verbaut sind:
 *
 *   IMU (BMI088)        - Telemetrie in allen Zustaenden
 *   Motor 1 + 2         - HDRM-Antriebe mit Encoder, per Telecommand nur in TEST
 *   Kraftsensor 1       - 3x HX711 (X/Y/Z), Telemetrie in allen Zustaenden
 *   Kraftsensor 2       - 4x HX711 (A/B/C/D), Verrechnung erst am Boden
 *   Down-/Uplink        - Serial4 (Pin 16/17) zum REXUS-Servicemodul
 *   REXUS-Signale       - werden eingelesen und als Rohpegel heruntergefunkt
 *   Roboterarm (RCU)    - Serial1 (Pin 0/1), Telemetrie durchgereicht,
 *                         Einzelgelenk-Jog per Telecommand nur in TEST
 *
 * Weitere Komponenten (Kameras, Temperatur) kommen Stueck fuer
 * Stueck dazu, sobald sie am Aufbau haengen und getestet werden.
 */

class System {
public:
    System();
    ~System();

    /**
     * @brief Initialisiert das gesamte System
     * @param after_watchdog_reset true, wenn der letzte Reset vom Watchdog kam
     * @return true wenn erfolgreich
     */
    bool init(bool after_watchdog_reset);

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
    MotorHAL* motor2_ = nullptr;                        ///< Motor 2 (baugleich zu Motor 1)
    ForceHAL* force1_ = nullptr;                        ///< Kraftsensor 1 (3x HX711, gemeinsamer Takt)
    ForceHAL* force2_ = nullptr;                        ///< Kraftsensor 2 (4x HX711, gemeinsamer Takt)
    UplinkReceiver* uplink_ = nullptr;                 ///< Telecommand-Empfang (Serial4 RX)
    REXUSHAL* rexus_ = nullptr;                         ///< REXUS-Signale L0/SOE/SODS
    ArmLink* arm_ = nullptr;                            ///< UART zur RCU des Roboterarms (Serial1)
    ArmTrajPlayer arm_traj_;                            ///< Trajektorien-Test an einem Gelenk (nur TEST)
    PersistentState persist_;                           ///< Zustand + Boot-Zaehler (EEPROM)
    SdLogger sd_;                                       ///< Bordspeicher (SD-Karte)

    // -----------------------------------------------------------------------
    // Zustandsmaschine
    // -----------------------------------------------------------------------
    StateMachine state_machine_;                        ///< Boden- und Flugzustaende
    MissionState last_state_ = MissionState::PRE_LAUNCH; ///< für die Zustandswechsel-Erkennung

    // -----------------------------------------------------------------------
    // Flugablauf (Zeiten: mission_timeline.hpp)
    // -----------------------------------------------------------------------
    uint8_t  flight_flags_      = 0;   ///< FLIGHT_*-Bits, gespiegelt im EEPROM
    uint16_t last_checkpoint_s_ = 0;   ///< zuletzt gesicherte Flugzeit in s

    /// Eine geplante HDRM-Fahrt. HDRM 2 startet HDRM_STAGGER_MS nach HDRM 1.
    struct HdrmMove {
        bool     pending = false;
        uint32_t due_ms  = 0;
        int16_t  degrees = 0;
    };
    HdrmMove hdrm_move_[2];            ///< [0] = HDRM 1 (motor_), [1] = HDRM 2 (motor2_)

    uint32_t last_telemetry_ms_ = 0;        ///< Zeitstempel des letzten IMU-Frames
    uint32_t last_motor_telemetry_ms_ = 0;  ///< Zeitstempel der letzten Motor-Frames
    uint8_t  last_event_signals_ = 0;       ///< REXUS-Signale beim letzten Ereignis-Frame
    uint32_t last_sys_telemetry_ms_ = 0;    ///< Zeitstempel des letzten SYS/STATE-Downlinks
    uint32_t last_arm_joint_ms_ = 0;        ///< Zeitstempel der letzten ARM/JOINT-Runde
    uint32_t last_arm_imu_ms_ = 0;          ///< Zeitstempel der letzten RCU-IMU-Frames
    uint32_t last_arm_power_ms_ = 0;        ///< Zeitstempel der letzten Spannungs-/Temperatur-Runde

    // -----------------------------------------------------------------------
    // Roboterarm
    // -----------------------------------------------------------------------
    /// Groesster Jog-Schritt pro Telecommand. Am Boden traegt ein Gelenk den
    /// Arm nicht - getestet wird nur ein einzelnes Gelenk in kleinen
    /// Schritten. Ein Tippfehler im Winkel soll das Gelenk nicht in den
    /// Anschlag fahren. Gegenstueck: ARM_JOG_MAX_DEG im Server.
    static constexpr int8_t ARM_JOG_MAX_DEG = 30;

    // -----------------------------------------------------------------------
    // Laufzeitbegrenzung des Motors
    // -----------------------------------------------------------------------
    // Der Motor laeuft Open-Loop und stoppt nicht mehr von selbst. Diese Grenze
    // ist die Rueckfallebene, falls MOTOR_OFF nicht durchkommt. 0 = aus.
    static constexpr uint32_t MOTOR_ON_TIMEOUT_MS = 30000;
    uint32_t motor_on_since_ms_  = 0;       ///< Zeitpunkt des letzten MOTOR_ON
    uint32_t motor2_on_since_ms_ = 0;       ///< Zeitpunkt des letzten MOTOR2_ON
    bool motor2_telemetry_first_ = false;   ///< Motor-Telemetrie abwechselnd zuerst senden

    // Kein Subsystem-Fehler ist fatal: der OBC laeuft degradiert weiter, statt
    // z.B. wegen eines fehlenden Motors auch die Telemetrie abzuschalten.
    // system_healthy fasst zusammen, ob ALLE Subsysteme oben sind - das Bit
    // geht per DL_STATUS1_SYSTEM_HEALTHY an die Bodenstation.
    bool imu_ready_ = false;                ///< IMU erfolgreich initialisiert
    bool downlink_ready_ = false;           ///< Downlink-UART bereit
    bool motor_ready_ = false;              ///< Motor (inkl. Encoder) initialisiert
    bool rexus_ready_ = false;              ///< REXUS-Signalpins konfiguriert
    bool arm_ready_ = false;                ///< Arm-UART offen (ob die RCU antwortet: arm_->linkAlive())
    bool force1_ready_ = false;             ///< Kraftsensor 1 initialisiert
    bool force2_ready_ = false;             ///< Kraftsensor 2 initialisiert
    bool motor2_ready_ = false;             ///< Motor 2 (inkl. Encoder) initialisiert

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

    /// Logische Motor-Aktion, unabhaengig vom Wire-Opcode (siehe execMotorAction).
    enum class MotorAction : uint8_t { OFF, ON, ZERO, TURN, GOTO };

    void printWelcomeBanner();
    void handleTelemetry();

    /// Ein MOTOR/STATE(2)-Frame fuer den angegebenen Motor senden, wenn bereit.
    void handleMotorTelemetry(MotorHAL* motor, bool ready, DownlinkMotorMsg msg);
    void handleForce();

    /// Einen Kraftsensor pollen und bei neuem Messwert (oder Haenger) senden.
    void handleForceSensor(ForceHAL* hal, bool ready, DownlinkForceMsg msg,
                           ForceChannelState& state, const char* label,
                           const uint8_t* pins, uint8_t sck);

    /// Nullabgleich eines Kraftsensors STARTEN (nicht blockierend). Das Ergebnis
    /// protokolliert handleForceSensor(), sobald der Abgleich fertig ist.
    void tareForceSensor(ForceHAL* hal, bool ready, const char* label);

    /// Pins und Nullpunkte eines Kraftsensors ans USB-Terminal melden.
    static void logForceSensor(ForceHAL* hal, const char* label,
                               const uint8_t* pins, uint8_t count, uint8_t sck);
    void handleSystemTelemetry(uint32_t now_ms);
    void handleStateMachine(uint32_t now_ms);
    /// Zustandswechsel seit dem letzten Aufruf uebernehmen (Speichern,
    /// Motoren, Meldung). Aus Telecommands UND aus update().
    void commitStateChange(uint32_t now_ms);
    /// Zustand/Signale melden: Ereignis-Frame (SD) + sofortiges SYS-Frame.
    void reportChange(MissionState previous, uint32_t now_ms);
    void onStateChanged(MissionState previous, MissionState current, uint32_t now_ms);

    /// Telecommand im aktuellen Zustand zulaessig? (Flug: keiner, siehe .cpp)
    bool commandAllowed(UplinkOpcode op) const;

    /// Aktionen des Flugablaufs: HDRM-Fahrten, Roboterarm, Motorsperre.
    void handleFlight(uint32_t now_ms);
    /// Beide HDRM-Fahrten einplanen (HDRM 2 um HDRM_STAGGER_MS versetzt).
    void scheduleHdrmMoves(uint32_t now_ms, int16_t degrees);
    /// Schliessen fuer DESCENT einplanen, falls die HDRMs geoeffnet wurden.
    void planDescent(uint32_t now_ms);
    /// Ereignis-Frame (nur SD) mit aktuellem Zustand und Signalen schreiben.
    void logStateEvent(MissionState previous, uint8_t prev_signals, uint32_t now_ms);
    /// FLIGHT_IGNORE_*-Bits in die Bitlage DL_REXUS_* umrechnen.
    static uint8_t eventSignals(uint8_t flight_bits);
    /// Aktuell anliegende REXUS-Signale als FLIGHT_IGNORE_*-Bits.
    uint8_t currentSignalBits() const;
    /// Sind beide Oeffnungsfahrten begonnen UND beendet?
    bool hdrmsOpenDone() const;
    /// Platzhalter: Roboterarm-Bewegungen in ROBOT_ARM.
    void robotArmStep(uint32_t t_ms);
    /// Flugzeit jede volle Sekunde ins EEPROM (nur bei stehenden Motoren).
    void updateCheckpoint(uint32_t now_ms);
    void handleUplink(uint32_t now_ms);
    bool handleMotorCommand(const UplinkCommand& cmd);

    /// Arm-Telemetrie in den Downlink (Raten: data_rates.hpp, ARM_*).
    void handleArmTelemetry(uint32_t now_ms);
    /// Arm-Telecommand (0x3x) ausfuehren. false = kein Arm-Opcode.
    /// Die Zustandspruefung (TEST-Sperre) macht der Aufrufer.
    bool handleArmCommand(const UplinkCommand& cmd);
    /// Gelenke anhalten und sperren - bei denselben Zustandswechseln wie die Motoren.
    void stopArm();

    /// Gemeinsame Kommandologik fuer Motor 1 und Motor 2 (siehe MotorAction).
    /// Motor-agnostisch: welcher Wire-Opcode auf welche Aktion/welchen Motor
    /// abbildet, entscheidet handleMotorCommand().
    bool execMotorAction(MotorHAL* motor, bool ready, const char* label,
                         MotorAction action, int16_t arg, uint32_t& on_since_ms);

    void handleMotorTimeout(uint32_t now_ms);

    /// true, solange einer der Motoren laeuft (dann keine SD-Zugriffe).
    bool motorsRunning() const;

    /// Laufzeitbegrenzung eines einzelnen Motors (siehe MOTOR_ON_TIMEOUT_MS).
    void checkMotorTimeout(MotorHAL* motor, bool ready, const char* label,
                           uint32_t on_since_ms, uint32_t now_ms);

    uint8_t subsystemBits() const;
};

// Externe System-Instanz (definiert in main.cpp)
extern System* g_system;
