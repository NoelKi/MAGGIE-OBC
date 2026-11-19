#pragma once

#include <cstdint>
#include <Arduino.h>
#include "hal/imu_hal.hpp"
#include "hal/force_hal.hpp"
#include "hal/arm_hal.hpp"

/**
 * @file telemetry_hal.hpp
 * @brief MAGGIE Downlink Telemetry (UART)
 *
 * Sends telemetry over a hardware UART using the fixed 20-byte MAGGIE
 * downlink frame. On the Teensy 4.1 the downlink is wired to pins
 * 16 (RX, updownlink+) / 17 (TX, updownlink-) -> Serial4.
 *
 * ---------------------------------------------------------------------------
 * Downlink frame (20 bytes, fixed size)
 * ---------------------------------------------------------------------------
 *  Idx | Field    | Bytes | Description
 *  ----+----------+-------+------------------------------------------------
 *   0  | START    |  1    | Frame start marker (DL_START)
 *   1  | MSGID1   |  1    | Subsystem / message category (DownlinkSubsystem)
 *   2  | MSGID2   |  1    | Message type within subsystem
 *   3  | ACK      |  1    | Acknowledge flag (0 for pure telemetry)
 *  4-5 | COUNTER  |  2    | Frame counter, big-endian, increments per frame
 *  6-7 | TIME     |  2    | On-board time, big-endian (lower 16 bit of millis())
 * 8-15 | DATA     |  8    | Payload, layout depends on MSGID (big-endian)
 *  16  | STATUS1  |  1    | Status bitfield 1
 *  17  | STATUS2  |  1    | Status bitfield 2
 *  18  | CRC      |  1    | CRC-8 (poly 0x07, init 0x00) over bytes 1..17
 *  19  | END      |  1    | Frame end marker (DL_END)
 *
 * All multi-byte fields are transmitted big-endian (most significant byte first).
 *
 * DATA layout per message type:
 *   IMU/ACCEL:   [ax_hi ax_lo ay_hi ay_lo az_hi az_lo  0 0]  (int16 sensor counts)
 *   IMU/GYRO :   [gx_hi gx_lo gy_hi gy_lo gz_hi gz_lo  0 0]  (int16 sensor counts)
 *   MOTOR/STATE: [pos(int32 BE) speed(int16 BE) state(uint8) 0]  (Encoder-Counts, PWM, Bits)
 *   SYS/STATE:   [state(uint8) subsys(uint8) uptime_ms(uint32 BE) 0 0]
 *   FORCE/TARGET1: [c0 c1 c2 0]      (3x int16: X, Y, Z)
 *   FORCE/TARGET2: [c0 c1 c2 c3]     (4x int16: A, B, C, D)
 *                  je int16 = TARIERTE HX711-Counts / FORCE_TELE_DIV
 *                  Flags beider Typen in STATUS2 (siehe DL_FORCE_*)
 *   ARM/JOINT:     [node state angle_urad(int32 BE) 0 0]
 *   ARM/VOLTAGE:   [node 0 bus_mv(uint16 BE) stepper_mv(uint16 BE) 0 0]
 *   ARM/TEMP:      [node 0 mcu(int16 BE) imu(int16 BE) 0 0]   (Rohwerte der RCU)
 *   ARM/RCU_ACCEL: [ax ay az 0 0]   (3x int16, +-4 g / 32768)
 *   ARM/RCU_GYRO : [gx gy gz 0 0]   (3x int16, +-500 dps / 32768)
 *   SYS/ARM_LINK:  [frames_ok(uint16 BE) frames_bad(uint16 BE) last_msg
 *                   last_node silence(uint8, 0,1 s) freefall_count]
 *                  STATUS2 = DL_ARM_LINK_*
 *   Die Arm-Werte reicht der OBC unveraendert von der RCU durch (siehe
 *   arm_hal.hpp), nur die Byte-Reihenfolge wird auf big-endian gedreht.
 *
 * The int16 values are the native BMI088 counts. The ground station applies
 * the documented scale factors:
 *   accel [m/s^2] = count * (9.80665 / 10920)   (+/-3 g range)
 *   gyro  [deg/s] = count * (1 / 65.536)         (+/-500 deg/s range)
 *
 * FORCE folgt demselben Muster: Der OBC schickt Counts, den Kalibrierfaktor
 * (Counts -> Newton) legt die Bodenstation an. Er ist am Boden aenderbar,
 * ohne die Firmware neu zu flashen - beim HX711 ist das der Wert, den man im
 * Labor als letztes festzurrt. Der Nullpunkt muss dagegen hier oben abgezogen
 * werden: Der Rohoffset des Wandlers liegt bei zehntausenden Counts und
 * spraengte das int16 sofort (siehe FORCE_TELE_DIV in force_hal.hpp).
 *
 * Bei TARGET2 gilt das doppelt: Dort sind die vier Kanaele Rohzellen eines
 * Eigenbaus, aus denen die Bodenstation per 3x4-Matrix erst einen Kraftvektor
 * rechnet. Waehlt man die Matrix falsch, laesst sich das aus den in InfluxDB
 * liegenden Rohkanaelen jederzeit neu auswerten - haette der OBC bereits
 * X/Y/Z gefunkt, waere die Messung verloren.
 *
 * WARUM DIE FLAGS IN STATUS2 STEHEN: TARGET2 braucht alle acht DATA-Bytes fuer
 * die vier int16. Damit beide FORCE-Typen dasselbe Format haben, liegen die
 * Flags auch bei TARGET1 in STATUS2 statt im DATA-Feld.
 */

/**
 * @brief Empfangsseite des Up-/Downlink-UART invertieren?
 *
 * Die TX-Seite (Downlink) ist NICHT betroffen - sie funktioniert fehlerfrei.
 * Nur der Telecommand-Pfad vom Boden kommt bit-invertiert am OBC an.
 *
 * BELEG: Ein gesendetes SDC-Paket (Kopf EB 90 A5 08 06 7E 10 00) erschien an
 * der UART als 0A 53 D1 F3 03 DF FF FF. Eine Simulation der 8N1-Strecke
 * reproduziert daraus bei 38400 Baud und invertierter Leitung 6 der 8 Bytes
 * exakt (F3 03 DF FF FF in Folge) und sagt 21 decodierte Bytes je 24
 * gesendeter voraus - gemessen wurden 21-22. Die Baudrate stimmt also, die
 * Polaritaet nicht.
 *
 * BLEIBT TROTZDEM AUF false - RXINV ist hier KEINE gueltige Kompensation:
 * Zwischen den Paketen kommt kein einziges Byte an (rx_bytes steht still), die
 * Leitung ruht also auf HIGH; laege sie auf LOW, erzeugte die UART aus dem
 * Dauer-Startbit ~3840 Byte/s. Ruhepegel korrekt, nur die Datenphase gedreht -
 * das ist ein VERTAUSCHTES Adernpaar A/B am MAX3488, kein durchgehender
 * Inverter. RXINV wuerde auch den Ruhepegel umdrehen und die UART damit in
 * einen Dauer-Break zwingen, ohne saubere Flanke vor dem Paket.
 *
 * Die OBC-Platine selbst ist laut Schaltplan (U9, MAX3488xSA) richtig
 * beschaltet: A = TC+ from SM, B = TC- from SM, RO -> updownlink+ (Pin 16 RX),
 * DI -> updownlink- (Pin 17 TX). Die Vertauschung liegt also NICHT hier,
 * sondern im Kabel zum Service-/Testmodul.
 *
 * STOLPERSTELLE am DSUB-15 des Service Module (REXUS User Manual): die
 * Polaritaet der beiden Paare ist gegenlaeufig nummeriert -
 *     Pin  6 = TM+   Pin  7 = TM-      (Plus auf der NIEDRIGEREN Nummer)
 *     Pin 13 = TC-   Pin 14 = TC+      (Plus auf der HOEHEREN Nummer)
 * Wer nach Muster statt nach Beschriftung auflegt, verdrahtet den Downlink
 * richtig und den Uplink gedreht - exakt das gemessene Fehlerbild.
 * Also pruefen: DSUB 14 -> A, DSUB 13 -> B.
 *
 * Diese Konstante bleibt nur fuer den Fall einer echten, durchgehenden
 * Invertierung (Ruhepegel MIT gedreht).
 */
static constexpr bool UPLINK_RX_INVERTED = false;

// Frame geometry
static constexpr uint8_t DOWNLINK_FRAME_SIZE = 20;

/// Hoechstens so viele Byte normaler Sensordaten (IMU, Kraft, Motor) duerfen in
/// der UART-Warteschlange stehen - 120 Byte sind bei 38400 Baud ~31 ms. Bei
/// hoeherer Baudrate entsprechend groesser waehlen. SYS-Frames haben Vorrang
/// und nutzen den ganzen Puffer (siehe sendFrame).
static constexpr int DOWNLINK_NORMAL_QUEUE_MAX = 120;
static constexpr uint8_t DOWNLINK_DATA_SIZE  = 8;

// Frame markers
static constexpr uint8_t DL_START = 0x7E;
static constexpr uint8_t DL_END   = 0x7F;

// MSGID1 - subsystem / message category
enum class DownlinkSubsystem : uint8_t {
    IMU   = 0x01,
    MOTOR = 0x02,
    SYS   = 0x03,
    FORCE = 0x04,
    ARM   = 0x05,   ///< Roboterarm, durchgereicht von der RCU (arm_hal.hpp)
    EVENT = 0x10,   ///< NUR SD-Karte - geht nie ueber die UART (siehe logEvent)
};

// MSGID2 - message type for the EVENT subsystem (SD only)
enum class DownlinkEventMsg : uint8_t {
    STATE = 0x01,   ///< Zustand oder REXUS-Signal hat sich geaendert
};

// MSGID2 - message type for the IMU subsystem
enum class DownlinkImuMsg : uint8_t {
    ACCEL = 0x01,
    GYRO  = 0x02,
};

// MSGID2 - message type for the MOTOR subsystem
enum class DownlinkMotorMsg : uint8_t {
    STATE  = 0x01,   ///< Motor 1
    STATE2 = 0x02,   ///< Motor 2 - baugleich, eigenes Frame wie FORCE TARGET1/TARGET2
};

// MSGID2 - message type for the FORCE subsystem
enum class DownlinkForceMsg : uint8_t {
    TARGET1 = 0x01,   ///< 3 Zellen X/Y/Z
    TARGET2 = 0x02,   ///< 4 Zellen A/B/C/D (Eigenbau, 3x 120 Grad + Z)
};

// MSGID2 - message type for the ARM subsystem
enum class DownlinkArmMsg : uint8_t {
    JOINT     = 0x01,   ///< Zustand + Winkel eines Gelenks
    VOLTAGE   = 0x02,   ///< Bus- und Stepper-Spannung eines Gelenks
    TEMP      = 0x03,   ///< MCU- und IMU-Temperatur eines Gelenks
    RCU_ACCEL = 0x04,   ///< RCU-IMU Beschleunigung
    RCU_GYRO  = 0x05,   ///< RCU-IMU Drehrate
};

// MSGID2 - message type for the SYS subsystem
enum class DownlinkSysMsg : uint8_t {
    STATE      = 0x01,
    UPLINK     = 0x02,   ///< Empfangsstatistik des Telecommand-Uplinks
    UPLINK_RAW = 0x03,   ///< Rohe Kopfbytes des zuletzt empfangenen Uplink-Bursts
    ARM_LINK   = 0x04,   ///< Empfangsstatistik der Arm-UART (SYS = Vorrangspur)
    ARM_TRAJ   = 0x05,   ///< Status des Trajektorientests (ArmTrajPlayer)
    ARM_FLIGHT = 0x06,   ///< Status der Flug-Trajektorie (ArmFlightPlayer) + Freigabe
    CAMERA     = 0x07,   ///< Status der 4 Kameras (CameraHAL)
    CAMERA_INFO = 0x08,  ///< Geraeteinfo EINER Kamera (reihum, 1 Hz)
};

// SYS/CAMERA - Flags je Kamera (DATA[0..3] = Kamera 1..4)
static constexpr uint8_t DL_CAM_ALIVE      = 0x01;  ///< Antwort juenger als CAM_ALIVE_MS
static constexpr uint8_t DL_CAM_ANSWERED   = 0x02;  ///< seit dem Start mindestens einmal geantwortet
static constexpr uint8_t DL_CAM_FEAT_START = 0x04;  ///< kann START_RECORDING
static constexpr uint8_t DL_CAM_FEAT_STOP  = 0x08;  ///< kann STOP_RECORDING
static constexpr uint8_t DL_CAM_SENT_START = 0x10;  ///< zuletzt gesendet: START
static constexpr uint8_t DL_CAM_SENT_STOP  = 0x20;  ///< zuletzt gesendet: STOP
static constexpr uint8_t DL_CAM_RX_ANY     = 0x40;  ///< Diagnose: je ein Byte auf eine Abfrage empfangen
static constexpr uint8_t DL_CAM_RX_GARBAGE = 0x80;  ///< Diagnose: Bytes, aber keine gueltige Antwort

// SYS/ARM_FLIGHT - flags (DATA[1])
static constexpr uint8_t DL_ARM_FLIGHT_SWITCH   = 0x01;  ///< Flug-Schalter an
static constexpr uint8_t DL_ARM_FLIGHT_UG_OBC   = 0x02;  ///< OBC-IMU misst Schwerelosigkeit
static constexpr uint8_t DL_ARM_FLIGHT_UG_RCU   = 0x04;  ///< RCU-IMU misst Schwerelosigkeit
static constexpr uint8_t DL_ARM_FLIGHT_TABLE_OK = 0x08;  ///< eingebaute Tabelle geprueft und ok
static constexpr uint8_t DL_ARM_FLIGHT_RESUMED  = 0x10;  ///< nach Neustart fortgesetzt
static constexpr uint8_t DL_ARM_FLIGHT_STARTED  = 0x20;  ///< in diesem Flug gestartet (EEPROM)
static constexpr uint8_t DL_ARM_FLIGHT_DONE     = 0x40;  ///< in diesem Flug fertig (EEPROM)
static constexpr uint8_t DL_ARM_FLIGHT_HOLD     = 0x80;  ///< Bahn wartet auf ein nachhaengendes Gelenk

// SYS/ARM_LINK - flags, uebertragen in STATUS2
static constexpr uint8_t DL_ARM_LINK_ALIVE = 0x01;  ///< bit0: gueltiges Frame juenger als ARM_LINK_TIMEOUT_MS

// STATUS1 bit definitions
static constexpr uint8_t DL_STATUS1_SYSTEM_HEALTHY = 0x01;  ///< bit0: system healthy
static constexpr uint8_t DL_STATUS1_IMU_VALID      = 0x02;  ///< bit1: IMU reading valid

// MOTOR/STATE - state byte bit definitions (DATA[6])
// Bit 4 war HDRM_CLOSED der frueheren Positionsregelung und ist mit ihr
// entfallen. Bleibt reserviert.
static constexpr uint8_t DL_MOTOR_STATE_ON          = 0x01;  ///< bit0: Motor dreht (on())
static constexpr uint8_t DL_MOTOR_STATE_ENCODER_OK  = 0x02;  ///< bit1: Encoder angehaengt und wird gelesen
static constexpr uint8_t DL_MOTOR_STATE_TURNING     = 0x04;  ///< bit2: turnBy()-Drehung laeuft
static constexpr uint8_t DL_MOTOR_STATE_TURN_FAILED = 0x08;  ///< bit3: letzte Drehung abgebrochen

// FORCE/TARGET1 + FORCE/TARGET2 - flags, uebertragen in STATUS2
//
// Ein Bit pro Kanal, in derselben Reihenfolge wie im DATA-Feld:
//   TARGET1 -> bit0 = X, bit1 = Y, bit2 = Z   (bit3 ungenutzt)
//   TARGET2 -> bit0 = A, bit1 = B, bit2 = C, bit3 = D
//
// Die SAT-Bits sind kein Schoenheitsfehler, sondern der Unterschied zwischen
// "die Kraft war so gross" und "der Downlink konnte nicht mehr": Ein
// abgeschnittener Wert sieht in der Kurve aus wie ein Plateau. Bei TARGET2
// wiegt das schwerer als bei TARGET1 - dort geht jede Zelle in die Berechnung
// von X und Y ein, eine still begrenzte verdreht also den gesamten Vektor.
static constexpr uint8_t DL_FORCE_SAT_0 = 0x01;  ///< bit0: Kanal 0 begrenzt
static constexpr uint8_t DL_FORCE_SAT_1 = 0x02;  ///< bit1: Kanal 1 begrenzt
static constexpr uint8_t DL_FORCE_SAT_2 = 0x04;  ///< bit2: Kanal 2 begrenzt
static constexpr uint8_t DL_FORCE_SAT_3 = 0x08;  ///< bit3: Kanal 3 begrenzt
static constexpr uint8_t DL_FORCE_TARED = 0x10;  ///< bit4: Nullabgleich gueltig
static constexpr uint8_t DL_FORCE_STALE = 0x20;  ///< bit5: seit FORCE_STALE_MS keine neue Wandlung

// SYS/STATE - subsystem byte bit definitions (DATA[1])
static constexpr uint8_t DL_SUBSYS_IMU      = 0x01;  ///< bit0: IMU initialisiert
static constexpr uint8_t DL_SUBSYS_MOTOR    = 0x02;  ///< bit1: Motor 1 + Encoder initialisiert
static constexpr uint8_t DL_SUBSYS_DOWNLINK = 0x04;  ///< bit2: Downlink-UART offen
static constexpr uint8_t DL_SUBSYS_FORCE1   = 0x08;  ///< bit3: Kraftsensor 1 initialisiert
static constexpr uint8_t DL_SUBSYS_FORCE2   = 0x10;  ///< bit4: Kraftsensor 2 initialisiert
static constexpr uint8_t DL_SUBSYS_MOTOR2   = 0x20;  ///< bit5: Motor 2 + Encoder initialisiert

class TelemetryDownlink {
public:
    /**
     * @brief Constructor
     * @param serial Hardware serial port used for the downlink (e.g. Serial4)
     */
    explicit TelemetryDownlink(HardwareSerial& serial);

    /**
     * @brief Initialize the downlink UART
     * @param baudrate Serial speed (default 38400)
     * @return true if successful
     */
    bool init(uint32_t baudrate = 38400);

    /**
     * @brief Send the IMU telemetry for one reading.
     *
     * Emits two frames: one ACCEL frame and one GYRO frame.
     *
     * @param reading Latest IMU reading
     * @param status1 STATUS1 byte (see DL_STATUS1_* flags)
     * @param status2 STATUS2 byte
     */
    void sendImu(const IMUReading& reading, uint8_t status1 = 0, uint8_t status2 = 0);

    /**
     * @brief Send the current motor state as one MOTOR/STATE (oder STATE2) frame.
     *
     * @param msg      STATE (Motor 1) oder STATE2 (Motor 2)
     * @param position Encoder position in quadrature counts
     * @param speed    Current signed PWM speed (-255..255)
     * @param state    State bitfield (see DL_MOTOR_STATE_* flags)
     * @param status1  STATUS1 byte (see DL_STATUS1_* flags)
     * @param status2  STATUS2 byte
     */
    void sendMotor(DownlinkMotorMsg msg, int32_t position, int16_t speed, uint8_t state,
                   uint8_t status1 = 0, uint8_t status2 = 0);

    /**
     * @brief Send one FORCE frame (Kraftsensor 1 oder 2).
     *
     * Die belegten Kanaele stehen in reading.count; nicht belegte DATA-Bytes
     * bleiben 0. Die Flags gehen in STATUS2, weil TARGET2 alle acht DATA-Bytes
     * fuer seine vier Zellen braucht.
     *
     * @param msg     TARGET1 (3 Kanaele) oder TARGET2 (4 Kanaele)
     * @param reading Letzter Messwert von ForceHAL::read()
     * @param stale   true, wenn seit FORCE_STALE_MS keine neue Wandlung kam
     * @param tared   true, wenn der Nullabgleich gueltig ist
     * @param status1 STATUS1 byte (see DL_STATUS1_* flags)
     */
    void sendForce(DownlinkForceMsg msg, const ForceReading& reading,
                   bool stale, bool tared, uint8_t status1 = 0);

    /**
     * @brief Send the current mission state as one SYS/STATE frame.
     *
     * @param mission_state MissionState-Wert (siehe mission_state.hpp)
     * @param subsystems    Bitfeld der initialisierten Subsysteme (DL_SUBSYS_*)
     * @param uptime_ms     Laufzeit seit Boot in ms
     * @param status1       STATUS1 byte (see DL_STATUS1_* flags)
     * @param status2       STATUS2 byte
     */
    /**
     * @param rexus Rohe REXUS-Leitungspegel (siehe DL_REXUS_* in rexus_hal.hpp).
     *              Geht als DATA[6] mit - macht am Boden sichtbar, ob eine
     *              offene Leitung die Flugsequenz ausloest.
     */
    void sendSystem(uint8_t mission_state, uint8_t subsystems, uint32_t uptime_ms,
                    uint8_t status1 = 0, uint8_t status2 = 0, uint8_t rexus = 0);

    /**
     * @brief Empfangsstatistik des Uplinks senden (SYS/UPLINK).
     *
     * Macht am Boden unterscheidbar, ob ein Telecommand die UART gar nicht
     * erreicht (rx_bytes steigt nicht) oder nur nicht geparst wird
     * (rx_bytes steigt, frames_ok nicht).
     *
     * @param rx_bytes    Bytes, die am RX-Pin angekommen sind
     * @param frames_ok   gueltig geparste Command-Frames
     * @param frames_bad  verworfene Frame-Kandidaten
     * @param last_opcode zuletzt akzeptierter Opcode (0xFF = noch keiner)
     */
    void sendUplinkStats(uint32_t rx_bytes, uint16_t frames_ok,
                         uint16_t frames_bad, uint8_t last_opcode);

    /**
     * @brief Rohe Kopfbytes des letzten Uplink-Bursts senden (SYS/UPLINK_RAW).
     *
     * Zeigt am Boden, WAS an der UART ankommt. Erwartet waeren die ersten acht
     * Bytes des SDC-Pakets: EB 90 A5 <mcnt> <dest/len> 7E <opcode> 00.
     * Abweichungen trennen Baudraten-, Invertierungs- und Verkabelungsfehler.
     *
     * @param bytes Zeiger auf bis zu DOWNLINK_DATA_SIZE Bytes
     * @param len   tatsaechlich gefuellte Anzahl (geht als STATUS2 mit)
     */
    void sendUplinkRaw(const uint8_t* bytes, uint8_t len);

    /// ARM/JOINT - Zustand und Winkel eines Gelenks (Layout siehe Dateikopf).
    void sendArmJoint(uint8_t node, const ArmJoint& joint);

    /// ARM/VOLTAGE und ARM/TEMP eines Gelenks.
    void sendArmVoltage(uint8_t node, const ArmJoint& joint);
    void sendArmTemp(uint8_t node, const ArmJoint& joint);

    /// ARM/RCU_ACCEL + ARM/RCU_GYRO.
    void sendArmRcuImu(const ArmRcuImu& imu);

    /**
     * @brief SYS/ARM_LINK - Empfangsstatistik der Arm-UART.
     *
     * Laeuft als SYS-Frame in der Vorrangspur: Ob der Arm ueberhaupt
     * antwortet, muss auch bei voller Leitung am Boden ankommen.
     */
    void sendArmLink(const ArmLink& link, uint32_t now_ms);

    /**
     * @brief SYS/ARM_TRAJ: Status des Trajektorientests (ArmTrajPlayer).
     *
     * DATA: [phase result node step(uint16 BE) max_err(uint16 BE, 0,01 Grad,
     *        gesaettigt) runs]. Werte von phase/result: ArmTrajPlayer::Phase
     * bzw. ::Result. node = Gelenk, das den Abbruch ausgeloest hat (0 = keins).
     */
    void sendArmTraj(uint8_t phase, uint8_t result, uint8_t node, uint16_t step,
                     float max_err_deg, uint8_t runs);

    /**
     * @brief SYS/ARM_FLIGHT: Status der Flug-Trajektorie (ArmFlightPlayer).
     *
     * DATA: [phase flags fault_sticky step(uint16 BE) max_err(uint16 BE,
     *        0,01 Grad, gesaettigt) joints]
     * STATUS1 = fault_live, STATUS2 = unterstes Byte der Tabellen-CRC32.
     * phase: ArmFlightPlayer::Phase, flags: DL_ARM_FLIGHT_*, Fehlermasken:
     * Bit k = Tabellenspalte k. joints = Gelenke der eingebauten Tabelle.
     */
    void sendArmFlight(uint8_t phase, uint8_t flags, uint8_t fault_sticky, uint16_t step,
                       float max_err_deg, uint8_t joints, uint8_t fault_live, uint8_t crc_lo);

    /**
     * @brief SYS/CAMERA: DATA[0..3] Flags (DL_CAM_*) je Kamera, DATA[4..7]
     * Sekunden seit der letzten Antwort (255 = nie oder >= 255 s).
     * STATUS1/STATUS2 = 0.
     */
    void sendCamera(const uint8_t flags[4], const uint8_t age_s[4]);

    /**
     * @brief SYS/CAMERA_INFO: DATA [Kamera 1..4][Protokoll][Features LE (2)]
     * [Fehlversuche BE (2)][0][0]. Reihum eine Kamera pro Sekunde.
     */
    void sendCameraInfo(uint8_t cam, uint8_t protocol, uint16_t features, uint16_t probes_failed);

    /**
     * @brief Ereignis-Frame NUR fuer die SD-Karte (EVENT/STATE).
     *
     * Zustandswechsel und REXUS-Flanken auf die Millisekunde - der Downlink
     * kennt sie nur aus dem 1-Hz-SYS-Frame. Geht ausschliesslich an den Tap,
     * nie ueber die UART, und zaehlt den Frame-Zaehler NICHT hoch (sonst saehe
     * der Boden Luecken, die keine sind).
     *
     * DATA: [state prev_state signals prev_signals millis(uint32 BE)]
     *       signals = entprellte, interpretierte REXUS-Signale (Bit 0 LO,
     *       Bit 1 SOE, Bit 2 SODS - wie DL_REXUS_*, aber 1 = liegt an).
     */
    void logEvent(uint8_t state, uint8_t prev_state, uint8_t signals,
                  uint8_t prev_signals, uint32_t millis_now);

    /// Frames, die wegen voller UART nur auf der SD-Karte gelandet sind.
    uint32_t uartDropped() const { return uart_dropped_; }

    /**
     * @brief Mithoerer fuer gesendete Frames (z.B. der SD-Bordspeicher).
     *
     * Bekommt jedes Frame NACH dem Senden, byte-identisch. Aendert nichts an
     * dem, was ueber die Leitung geht.
     */
    using FrameTap = void (*)(void* ctx, const uint8_t* frame, uint8_t len);
    void setTap(FrameTap tap, void* ctx) { tap_ = tap; tap_ctx_ = ctx; }

private:
    HardwareSerial& serial_;
    FrameTap tap_     = nullptr;
    void*    tap_ctx_ = nullptr;
    uint16_t counter_ = 0;
    bool initialized_ = false;
    uint32_t uart_dropped_ = 0;
    int      tx_capacity_  = 0;   ///< groesster je gesehener freier UART-Puffer

    /**
     * @brief Assemble and transmit one 20-byte downlink frame.
     * @param data 8-byte payload (DOWNLINK_DATA_SIZE)
     */
    void sendFrame(uint8_t msgid1, uint8_t msgid2, uint8_t ack,
                   const uint8_t data[DOWNLINK_DATA_SIZE],
                   uint8_t status1, uint8_t status2, bool sd_only = false);

    /// CRC-8, polynomial 0x07, init 0x00 (SMBus/CCITT style)
    static uint8_t crc8(const uint8_t* data, size_t len);
};
