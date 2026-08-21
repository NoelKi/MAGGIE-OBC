#pragma once

#include <cstdint>
using std::uint8_t;
using std::uint32_t;

/**
 * @file pin_config.hpp
 * @brief Central pin assignment for Teensy 4.1 OBC
 *
 * Define all hardware pins in one place - so only this file needs to be
 * changed during rewiring.
 *
 * Enthaelt nur noch die Pins der tatsaechlich verbauten Komponenten. Kommen
 * weitere Sensoren dazu, gehoeren ihre Pins hier ergaenzt - die vollstaendige
 * Belegung des Boards steht in docs/teensyPins/MAGGIE-OCB-PIN-BELEGUNG.txt.
 *
 * Stand: deckungsgleich mit dieser Belegungstabelle. Frueher wich diese Datei
 * an vier Stellen davon ab (Motorkanal B, Encoder, L0_t, Up-/Downlink), weil
 * der Tischaufbau anders verdrahtet war - diese Ausweichbelegungen sind
 * entfallen. Wer an einem alten Aufbau misst, muss also umstecken.
 *
 * Source: Teensy 4.1 Pinout Diagram (MAGGIE OBC v1.6)
 */

// ===========================================================================
// SPI Bus (IMU)
// ===========================================================================
static constexpr uint8_t PIN_SPI1_MOSI = 11;   ///< MOSI
static constexpr uint8_t PIN_SPI1_MISO = 12;   ///< MISO
static constexpr uint8_t PIN_SCK = 13;         ///< SCK

// Chip Select Pins des BMI088 (getrennte Dies fuer Accel und Gyro)
static constexpr uint8_t PIN_CS_ACCEL = 37;    ///< Chip Select Accelerometer (CS_ACCEL)
static constexpr uint8_t PIN_CS_GYRO = 36;     ///< Chip Select Gyroscope (CS_GYRO)

// ===========================================================================
// Motor 1 - HDRM-Antrieb (DRV8871)
// ===========================================================================
// Belegungstabelle: 18 = M1_A, 14 = M1_B (Pololu 380/1 am DRV8871).
//
// Beide Pins haben auf der Teensy 4.1 einen QuadTimer3-Kanal (18 = Kanal 1,
// 14 = Kanal 2), MotorHAL taktet sie also per analogWrite() in Hardware mit
// 20 kHz - die Software-PWM (IntervalTimer, siehe motor_hal.hpp) greift hier
// nicht.
//
// Kanal A ist die Vorwaertsrichtung: setSpeed(+x) legt die PWM auf A, B bleibt
// LOW. Dreht der Motor verkehrt herum, die beiden Zeilen tauschen.
//
// Pin 19 ist laut Tabelle CAMDIR1 (Kamera-MUX, in dieser Firmware nicht
// instanziiert) und war nur die Behelfsverdrahtung des Tischaufbaus.
static constexpr uint8_t PIN_M1_A = 18;        ///< Motor 1 Channel A (vorwaerts, M1_A)
static constexpr uint8_t PIN_M1_B = 14;        ///< Motor 1 Channel B (rueckwaerts, M1_B)

// Motor 1 Quadratur-Encoder (A/B) - reine Positionsmessung, keine Regelung.
// Verbaut: Pololu enc03d (0J12461) am Getriebemotor.
//
// Belegungstabelle: 5 = ENC_HDRM1A, 6 = ENC_HDRM1B. Auf dem Teensy 4.1 ist
// jeder Digitalpin interruptfaehig, die Encoder-Bibliothek arbeitet hier
// normal.
//
// Die frueher benutzten Pins 0/1 sind die ARM-UART (Serial1) und waren nur
// deshalb frei, weil der Roboterarm in dieser Firmware nirgends instanziiert
// wird. Mit 5/6 entfaellt diese Kollision.
static constexpr uint8_t PIN_M1_ENC_A = 5;     ///< Motor 1 Encoder Channel A (ENC_HDRM1A)
static constexpr uint8_t PIN_M1_ENC_B = 6;     ///< Motor 1 Encoder Channel B (ENC_HDRM1B)

// ===========================================================================
// Kraftsensoren - je eine HX711-Gruppe an EINEM gemeinsamen Takt
// ===========================================================================
// WICHTIG fuer beide Gruppen: Die Wandler teilen sich die Taktleitung. Das ist
// kein Fehler der Verdrahtung, sondern die uebliche Bauform - es zwingt aber
// dazu, alle DOUT-Leitungen einer Gruppe in DERSELBEN Taktschleife abzutasten
// (siehe force_hal.hpp). Ein Treiber, der pro Zelle taktet, schiebt die Daten
// der uebrigen mit heraus und verwirft sie.
//
// Die REIHENFOLGE in den Arrays unten ist die Kanalnummerierung im Downlink.
// Wird sie vertauscht, rechnet die Bodenstation mit den falschen Zellen.

// --- Kraftsensor 1 (Target 1), 3 Zellen X/Y/Z -----------------------------
// Belegungstabelle: 4 = Force_X_1, 3 = Force_Y_1, 2 = Force_Z_1, 27 = Force_CLK_1
static constexpr uint8_t PIN_FORCE1_DOUT[3] = { 4, 3, 2 };  ///< X, Y, Z
static constexpr uint8_t PIN_FORCE1_SCK     = 27;           ///< Force_CLK_1

// --- Kraftsensor 2 (Target 2), 4 Zellen A/B/C/D ---------------------------
// Belegungstabelle: 30 = Force_2_A, 31 = Force_2_B, 32 = Force_2_C,
// 41 = Force_2_D, 9 = Force_CLK_2.
//
// Eigenbau: A, B und C stehen um 120 Grad versetzt in der XY-Ebene, D haengt
// direkt in Z. Die Umrechnung der vier Zellen in einen Kraftvektor passiert
// NICHT hier, sondern in der Bodenstation
// (MAGGIE_SERVER/app/services/downlink_frame_parser.py) - der OBC funkt nur
// die vier tarierten Rohkanaele. Grund: Die Geometriefaktoren stammen aus
// einer Kalibriermessung und aendern sich beim Umbau; am Boden sind sie ohne
// Neuflashen anzupassen, und die Rohkanaele bleiben in InfluxDB erhalten,
// sodass sich eine falsch kalibrierte Messung nachtraeglich neu auswerten
// laesst.
static constexpr uint8_t PIN_FORCE2_DOUT[4] = { 30, 31, 32, 41 };  ///< A, B, C, D
static constexpr uint8_t PIN_FORCE2_SCK     = 9;                   ///< Force_CLK_2

// ===========================================================================
// REXUS Signals
// ===========================================================================
// Belegungstabelle: 40 = LO_T, 39 = SOE_T, 38 = SODS_T.
//
// L0_t lag zwischenzeitlich auf Pin 21 (in der Tabelle CAM1_TX), weil der
// Motor damals seinen Kanal B auf Pin 40 hatte und REXUSHAL::init() den Pin
// nach dem Motor als Eingang zurueckkonfiguriert haette - der Motorkanal waere
// tot gewesen. Der Motor sitzt auf 18/14, damit ist Pin 40 frei.
static constexpr uint8_t PIN_L0_T = 40;        ///< L0_t Signal (LO_T)
static constexpr uint8_t PIN_SOE_I = 39;       ///< SOE_i Signal (SOE_T)
static constexpr uint8_t PIN_SODS_I = 38;      ///< SODS_i Signal (SODS_T)

/**
 * @brief Pegel, bei dem ein REXUS-Signal als ausgeloest gilt.
 *
 * true  = aktiv HIGH (Signal liegt direkt am Pin an). REXUSHAL zieht die
 *         Leitungen dann per INPUT_PULLDOWN auf LOW, damit eine offene
 *         Leitung als "nicht ausgeloest" gilt.
 * false = aktiv LOW. Fuer eine invertierende Eingangsstufe (Optokoppler oder
 *         Pegelwandler mit Pull-up), deren Ausgang im Ruhezustand auf HIGH
 *         liegt. REXUSHAL zieht die Leitungen dann per INPUT_PULLUP hoch,
 *         damit auch hier eine offene Leitung "nicht ausgeloest" bedeutet.
 *
 * EINGESTELLT AUF AKTIV LOW - so sieht die Schnittstelle laut Schaltplan der
 * Elektrotechniker aus (Open Drain):
 *
 *   Experiment-Seite:     Pull-up nach VCC, Abgriff zum Mikrocontroller
 *   Service-Module-Seite: NMOS, Drain an der Leitung, Source an GND,
 *                         Gate = "Input from Service Module"
 *
 *   Signal liegt an  -> Gate HIGH -> NMOS leitet -> Leitung auf GND = LOW
 *   Signal ruht      -> NMOS sperrt -> Pull-up haelt die Leitung auf HIGH
 *
 * Deckt sich mit der Messung am Bodenaufbau: Pin 38 (SODS) und 39 (SOE) lagen
 * dauerhaft HIGH (= Ruhe), waehrend der damals unbeschaltete L0-Pin LOW
 * lieferte.
 *
 * ACHTUNG Hardware: Die Teensy-4.1-GPIOs sind NICHT 5-V-tolerant. Der Pull-up
 * auf der Experiment-Seite muss nach 3,3 V gehen, nicht nach 5 V.
 */
static constexpr bool REXUS_ACTIVE_HIGH = false;

// ===========================================================================
// Up/Down-link (Serial4, RS-422 zum REXUS-Servicemodul)
// ===========================================================================
// Belegungstabelle: 16 = Up/Downlink+, 17 = Up/Downlink-. Das ist auf der
// Teensy 4.1 Serial4 (16 = RX4, 17 = TX4) - NICHT Serial8 (34/35), auf dem
// der Tischaufbau frueher lief. Wer den Port wechselt, muss auch die
// Serial4-Instanz in src/system.cpp mitziehen.
//
// Beide Pins haben keinen PWM-Timer (siehe MotorHAL::pinHasHardwarePwm) - fuer
// eine UART irrelevant, aber sie taugen deshalb nicht als Motorkanal.
static constexpr uint8_t PIN_UPDOWNLINK_MINUS = 17;  ///< updownlink- (Serial4 TX)
static constexpr uint8_t PIN_UPDOWNLINK_PLUS = 16;   ///< updownlink+ (Serial4 RX)
