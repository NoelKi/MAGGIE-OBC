#pragma once
#include <cstdint>
#include "hal/imu_hal.hpp"
#include "hal/telemetry_hal.hpp"
#include "hal/motor_hal.hpp"
#include "hal/force_hal.hpp"
#include "hal/uplink_hal.hpp"
#include "hal/rexus_hal.hpp"
#include "hal/arm_hal.hpp"
#include "hal/camera_hal.hpp"
#include "arm_traj_player.hpp"
#include "arm_flight_player.hpp"
#include "microgravity_detector.hpp"
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
    CameraHAL* cam_ = nullptr;                          ///< 4 Kameras an Serial5 ueber Mux
    ArmTrajPlayer arm_traj_;                            ///< Trajektorien-Test an einem Gelenk (nur TEST)
    ArmFlightPlayer arm_flight_{builtinFlightTable()};  ///< Flug-Trajektorie (ROBOT_ARM)
    MicrogravityDetector ug_obc_;                       ///< Schwerelosigkeit aus der OBC-IMU
    MicrogravityDetector ug_rcu_;                       ///< Schwerelosigkeit aus der RCU-IMU
    uint32_t rcu_imu_seen_ms_ = 0;                      ///< updated_ms des zuletzt eingespeisten RCU-IMU-Werts
    bool arm_flight_enabled_ = false;                   ///< Flug-Schalter (ARM_FLIGHT_ENABLE), gespiegelt im EEPROM
    bool arm_flight_refused_logged_ = false;            ///< "Tabelle nicht ok" nur einmal melden
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

    /// Ein HDRM oeffnen/schliessen - so oft, bis der Encoder das Ziel meldet
    /// (Nutzer, 19.11.2026: "es MUSS sich oeffnen"). Nach jedem Fehlversuch
    /// wackelt der Motor im Getriebespiel, dann kommt der naechste Versuch.
    struct HdrmCtl {
        enum class Phase : uint8_t { IDLE, WAIT, MOVING, WIGGLE, OPEN_LOOP, DONE };
        Phase    phase    = Phase::IDLE;
        bool     open     = true;   ///< Ziel offen (true) oder zu
        bool     blind    = false;  ///< Ausgangslage unbekannt (Neustart mitten im Ablauf)
        long     target   = 0;      ///< Encoder-Ziel in Counts
        long     ref_pos  = 0;      ///< Bezug fuer "wie weit gedreht"
        long     try_pos  = 0;      ///< Position zu Beginn des laufenden Versuchs
        uint32_t next_ms  = 0;      ///< naechster Versuch fruehestens / Ende OPEN_LOOP
        uint16_t attempts = 0;
        uint8_t  no_move  = 0;      ///< Versuche in Folge ohne jede Encoder-Bewegung
    };
    HdrmCtl hdrm_[2];                  ///< [0] = HDRM 1 (motor_), [1] = HDRM 2 (motor2_)
    long    hdrm_closed_pos_[2] = {};  ///< Encoder-Position "zu" (Start des Oeffnens)
    /// Bit i = HDRM i+1 gilt nur auf Zeit / ohne Motor als offen (kein Encoder-
    /// Beweis). Dann wird die Arm-Trajektorie abgebrochen, sobald der Arm
    /// ARM_BLIND_STUCK_MS lang nicht vorankommt (Nutzer, 19.11.2026). Im EEPROM.
    uint8_t hdrm_blind_ = 0;
    uint32_t arm_stuck_since_ms_ = 0;          ///< 0 = Arm kommt voran
    static constexpr uint32_t ARM_BLIND_STUCK_MS = 2000;
    bool    hdrm_closed_known_[2] = {};///< false nach einem Neustart mitten im Flug

    /// Nach einem Neustart mitten im Oeffnen (Lage unbekannt): spaetestens nach
    /// so vielen Versuchen gilt das HDRM als offen - der Arm soll starten.
    static constexpr uint8_t  HDRM_BLIND_MAX_ATTEMPTS = 3;
    /// So viele Versuche in Folge ohne jede Encoder-Bewegung = Encoder tot
    /// (oder Mechanik fest): dann einmal auf Zeit fahren und als fertig werten.
    static constexpr uint8_t  HDRM_ENCODER_DEAD_TRIES = 3;
    static constexpr long     HDRM_NO_MOVE_COUNTS     = 20;
    /// Fahrt auf Zeit ohne Encoder: Motor 1 brauchte 1,1 s fuer 165 Grad.
    static constexpr uint32_t HDRM_OPEN_LOOP_MS    = 1500;
    static constexpr int16_t  HDRM_OPEN_LOOP_SPEED = 220;
    /// Pause vor Versuch n: n x HDRM_RETRY_PAUSE_MS, hoechstens HDRM_RETRY_PAUSE_MAX_MS.
    static constexpr uint32_t HDRM_RETRY_PAUSE_MS     = 500;
    static constexpr uint32_t HDRM_RETRY_PAUSE_MAX_MS = 3000;

    uint32_t last_telemetry_ms_ = 0;        ///< Zeitstempel des letzten IMU-Frames
    uint32_t last_motor_telemetry_ms_ = 0;  ///< Zeitstempel der letzten Motor-Frames
    uint8_t  last_event_signals_ = 0;       ///< REXUS-Signale beim letzten Ereignis-Frame
    uint32_t last_sys_telemetry_ms_ = 0;    ///< Zeitstempel des letzten SYS/STATE-Downlinks
    uint32_t last_arm_joint_ms_ = 0;        ///< Zeitstempel der letzten ARM/JOINT-Runde
    uint32_t last_arm_imu_ms_ = 0;          ///< Zeitstempel der letzten RCU-IMU-Frames
    uint32_t last_arm_power_ms_ = 0;        ///< Zeitstempel der letzten Spannungs-/Temperatur-Runde
    uint32_t last_sd_motor_ms_ = 0;         ///< Zeitstempel der letzten Motor-Zeile auf der SD-Karte
    uint32_t last_dl_imu_ms_ = 0;           ///< Zeitstempel des letzten IMU-Frames im Downlink
    uint32_t last_status_ms_ = 0;           ///< Zeitstempel des letzten SYS/STATE-Frames
    uint32_t last_diag_ms_ = 0;             ///< Zeitstempel des letzten Diagnose-Slots
    uint8_t  diag_slot_ = 0;                ///< naechster Diagnose-Slot (sendDiagFrame)
    static constexpr uint8_t DIAG_SLOTS = 8; ///< Diagnose-Frames reihum, je 1x pro SYS_INTERVAL_MS
    uint8_t  arm_power_step_ = 0;           ///< Spannung/Temperatur reihum: Gelenk = step/2 + 1
    uint8_t  arm_joint_next_ = 1;           ///< naechstes Gelenk fuer ARM/JOINT
    uint32_t last_loop_max_us_ = 0;         ///< Loop-Maximum der letzten SYS-Periode
    uint32_t last_arm_report_ms_ = 0;       ///< Arm-UART-Zusammenfassung im Log
    uint32_t arm_report_ok_ = 0, arm_report_crc_ = 0, arm_report_id_ = 0;

    // Loop-Statistik je SYS-Periode (SYS.csv, SYS/BOOT)
    uint32_t loop_max_us_ = 0;
    uint32_t loop_count_ = 0;
    uint32_t loop_stats_since_ms_ = 0;

    // -----------------------------------------------------------------------
    // Roboterarm
    // -----------------------------------------------------------------------
    /// Groesster Jog-Schritt pro Telecommand. Am Boden traegt ein Gelenk den
    /// Arm nicht - getestet wird nur ein einzelnes Gelenk in kleinen
    /// Schritten. Ein Tippfehler im Winkel soll das Gelenk nicht in den
    /// Anschlag fahren. Gegenstueck: ARM_JOG_MAX_DEG im Server.
    static constexpr int8_t ARM_JOG_MAX_DEG = 30;
    /// Hoechste Geschwindigkeit/Beschleunigung fuer ARM_SET_VELOCITY (Vorgabe
    /// Team, 15.11.2026: 10 Grad/s, 4 Grad/s^2). Gegenstueck:
    /// ARM_SPEED_MAX_DEG_S im Server und ARM_SPEED_MAX in der GS.
    static constexpr uint8_t ARM_JOG_MAX_VEL_DEG_S    = 10;
    static constexpr uint8_t ARM_JOG_MAX_ACCEL_DEG_S2 = 4;

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
    bool cam_ready_ = false;                ///< Kamera-UART offen (ob sie antworten: SYS/CAMERA)
    uint8_t cam_info_next_ = 0;             ///< naechste Kamera fuer SYS/CAMERA_INFO

    /**
     * Kameras nehmen selbst ab dem Einschalten auf (Kamera-Einstellung).
     * Zur Sicherheit schickt der OBC beim Eintritt in ARMED, ASCENT und
     * EXPERIMENT sowie nach einem Neustart im Flug START_RECORDING an alle.
     * Ein STOP geht nie automatisch raus - die Kameras laufen bis Strom aus
     * (vom Nutzer am 17.11.2026 so festgelegt).
     *
     * VOR DEM FLUG AM BODEN PRUEFEN: START an eine Kamera, die schon
     * aufnimmt, darf die Aufnahme NICHT beenden. Tut sie es doch, hier auf
     * false setzen.
     */
    static constexpr bool CAM_SAFETY_START = true;

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
        uint32_t last_tx_ms = 0;            ///< Zeitstempel des letzten Werts (SD, STALE-Takt)
        uint32_t last_dl_ms = 0;            ///< Zeitstempel des letzten FORCE-Frames im Downlink
    };
    ForceChannelState force1_state_;
    ForceChannelState force2_state_;

    /// Sendeintervall, solange ein Sensor haengt. Ohne diese Bremse ginge bei
    /// stehendem Wandler in JEDEM Loop ein Frame raus (siehe handleForceSensor).
    static constexpr uint32_t FORCE_STALE_TX_INTERVAL_MS = 1000;

    /// Logische Motor-Aktion, unabhaengig vom Wire-Opcode (siehe execMotorAction).
    enum class MotorAction : uint8_t { OFF, ON, ZERO, TURN, GOTO };

    void printWelcomeBanner();
    void handleTelemetry(uint32_t now_ms);

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
    /// Stufe 1: SYS/STATE mit Zustand, HDRM, Arm/SD/Kameras live (2 Hz + bei Wechsel).
    void sendStatusFrame(uint32_t now_ms);
    /// Stufe 1: ein Diagnose-Slot (Uplink, Arm-Link, Arm, Kameras, SD, Boot) reihum.
    void sendDiagFrame(uint32_t now_ms);
    /// Alle 10 s: Arm-UART ok / CRC-Fehler / unbekannte ID ins Log.
    void reportArmLink(uint32_t now_ms);
    /// 1-Hz-Zeile fuer SYS.csv und je Kamera eine Zeile fuer CAMERA.csv.
    void logSystemToSd(uint32_t now_ms, uint8_t rexus_bits, uint32_t loop_max_us,
                       uint32_t loops_per_s);
    /// Motorzustand fuer MOTOR.csv (Takt SD_MOTOR_INTERVAL_MS).
    void logMotorToSd(MotorHAL* motor, bool ready, uint8_t number);
    /// DL_MOTOR_STATE_*-Bits eines Motors.
    static uint8_t motorStateBits(MotorHAL* motor);
    /// DL_CAM_*-Bits einer Kamera (0..3).
    uint8_t cameraFlags(uint8_t cam, uint32_t now_ms) const;
    /// Nach einem Watchdog-Reset: Schuldigen und Abgeschaltetes melden.
    void logBootGuard();
    /// Kopf von INFO.txt (Boot-Nr., Firmware, Reset-Grund).
    void writeSdInfo(bool after_watchdog_reset);
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
    /// Beide HDRMs oeffnen bzw. schliessen (HDRM 2 um HDRM_STAGGER_MS versetzt).
    void startHdrms(uint32_t now_ms, bool open, bool blind = false);
    /// HDRM-Ablauf: Versuch starten, Ergebnis pruefen, wackeln, wiederholen.
    void updateHdrm(uint8_t i, uint32_t now_ms);
    /// Laufende HDRM-Ablaeufe abbrechen (Rueckkehr an den Boden, Flugbeginn).
    void cancelHdrms();
    /// Schliessen fuer DESCENT einplanen, falls die HDRMs geoeffnet wurden.
    void planDescent(uint32_t now_ms);
    /// Ereignis-Frame (nur SD) mit aktuellem Zustand und Signalen schreiben.
    void logStateEvent(MissionState previous, uint8_t prev_signals, uint32_t now_ms);
    /// FLIGHT_IGNORE_*-Bits in die Bitlage DL_REXUS_* umrechnen.
    static uint8_t eventSignals(uint8_t flight_bits);
    /// Aktuell anliegende REXUS-Signale als FLIGHT_IGNORE_*-Bits.
    uint8_t currentSignalBits() const;
    /// Sind beide HDRMs NACHWEISLICH offen (Encoder am Ziel)?
    bool hdrmsOpenDone() const;
    /// Flug-Trajektorie des Arms in ROBOT_ARM (Start) und DESCENT (nur
    /// Fortsetzen/Beenden). Freigabe: armFlightGateOpen().
    void armFlightStep(uint32_t now_ms, MissionState state);
    /// Flug-Schalter an ODER eine IMU misst Schwerelosigkeit.
    bool armFlightGateOpen(uint32_t now_ms) const;
    /// DL_ARM_FLIGHT_*-Bits fuer SYS/ARM_FLIGHT.
    uint8_t armFlightFlags(uint32_t now_ms) const;
    /// SYS/ARM_FLIGHT in den Downlink.
    void sendArmFlightStatus(uint32_t now_ms);
    void sendCameraStatus(uint32_t now_ms);
    void cameraSafetyStart(const char* why);
    /// Telecommand ARM_FLIGHT_ENABLE ausfuehren.
    void setArmFlightEnable(bool enable);
    /// Flugzeit jede volle Sekunde ins EEPROM (nur bei stehenden Motoren).
    void updateCheckpoint(uint32_t now_ms);
    void handleUplink(uint32_t now_ms);
    bool handleMotorCommand(const UplinkCommand& cmd);

    /// Arm-Telemetrie in den Downlink (Raten: data_rates.hpp, ARM_*).
    void handleArmTelemetry(uint32_t now_ms);
    /// SYS/ARM_TRAJ: Status des Trajektorientests in den Downlink.
    void sendArmTrajStatus();
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
