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
 * Source: Teensy 4.1 Pinout Diagram (MAGGIE OBC v1.6)
 */

// ===========================================================================
// SPI Bus (IMU)
// ===========================================================================
static constexpr uint8_t PIN_SPI1_MOSI = 11;   ///< MOSI
static constexpr uint8_t PIN_SPI1_MISO = 12;   ///< MISO
static constexpr uint8_t PIN_SCK = 13;         ///< SCK

// Chip Select Pins des BMI088 (getrennte Dies fuer Accel und Gyro)
static constexpr uint8_t PIN_CS_ACCEL = 37;    ///< Chip Select Accelerometer
static constexpr uint8_t PIN_CS_GYRO = 36;     ///< Chip Select Gyroscope

// ===========================================================================
// Motor 1 - HDRM-Antrieb (DRV8871)
// ===========================================================================
// Beide Pins haben auf der Teensy 4.1 einen QuadTimer-Kanal, MotorHAL taktet
// sie also per analogWrite() in Hardware mit 20 kHz - die Software-PWM
// (IntervalTimer, siehe motor_hal.hpp) greift hier nicht.
//
// Kanal A ist die Vorwaertsrichtung: setSpeed(+x) legt die PWM auf A, B bleibt
// LOW. Dreht der Motor verkehrt herum, die beiden Zeilen tauschen.
//
// ACHTUNG: In docs/teensyPins/MAGGIE-OCB-PIN-BELEGUNG.txt liegt M1_B auf Pin 14,
// Pin 19 ist dort CAMDIR1 (Kamera-MUX, in dieser Firmware nicht instanziiert).
// 18/19 ist die Verdrahtung des Tischaufbaus (wie in hardwareTest/motor.cpp);
// vor dem Flug gegen die Platine abgleichen.
static constexpr uint8_t PIN_M1_A = 18;        ///< Motor 1 Channel A (vorwaerts)
static constexpr uint8_t PIN_M1_B = 19;        ///< Motor 1 Channel B (rueckwaerts)

// Motor 1 Quadratur-Encoder (A/B) - reine Positionsmessung, keine Regelung.
// Verbaut: Pololu enc03d (0J12461) am Getriebemotor.
//
// Pin 0/1 sind in der Belegungstabelle die ARM-UART (Serial1). Der Roboterarm
// wird in dieser Firmware nirgends instanziiert - es gibt kein Serial1.begin().
// Die Pins sind damit reines GPIO und frei. Auf dem Teensy 4.1 ist jeder
// Digitalpin interruptfaehig, die Encoder-Bibliothek arbeitet hier normal.
//
// ACHTUNG: Sobald die ARM-Kommunikation dazukommt, kollidiert sie hier - dann
// muss der Encoder umziehen. Laut Belegungstabelle sind 5+6 die eigentlichen
// Encoder-Pins von Motor 1 (ENC_HDRM1A/B).
static constexpr uint8_t PIN_M1_ENC_A = 0;     ///< Motor 1 Encoder Channel A
static constexpr uint8_t PIN_M1_ENC_B = 1;     ///< Motor 1 Encoder Channel B

// ===========================================================================
// REXUS Signals
// ===========================================================================
// L0_t liegt laut Belegungstabelle auf Pin 40. Als der Motor dort noch seinen
// Kanal B hatte, kollidierten die beiden: REXUSHAL::init() laeuft in
// System::init() NACH dem Motor und haette den Pin als Eingang
// zurueckkonfiguriert - der Motorkanal waere tot gewesen. Deshalb das
// Ausweichen auf Pin 21.
//
// Seit der Motor auf 18/19 sitzt, ist Pin 40 wieder frei; L0_t koennte also
// zurueck auf seinen dokumentierten Pin. Bewusst noch nicht umgestellt, weil
// der Tischaufbau auf 21 verdrahtet ist. Pin 21 ist in der Tabelle CAM1_TX -
// sobald die Kamera dazukommt, muss L0_t ohnehin zurueck auf 40.
// TODO: vor dem Flug gegen docs/teensyPins/MAGGIE-OCB-PIN-BELEGUNG.txt abgleichen.
static constexpr uint8_t PIN_L0_T = 21;        ///< L0_t Signal
static constexpr uint8_t PIN_SOE_I = 39;       ///< SOE_i Signal
static constexpr uint8_t PIN_SODS_I = 38;      ///< SODS_i Signal

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
 * dauerhaft HIGH (= Ruhe), waehrend der unbeschaltete Pin 21 LOW lieferte.
 *
 * ACHTUNG Hardware: Die Teensy-4.1-GPIOs sind NICHT 5-V-tolerant. Der Pull-up
 * auf der Experiment-Seite muss nach 3,3 V gehen, nicht nach 5 V.
 */
static constexpr bool REXUS_ACTIVE_HIGH = false;

// ===========================================================================
// Up/Down-link (Serial8, RS-422 zum REXUS-Servicemodul)
// ===========================================================================
static constexpr uint8_t PIN_UPDOWNLINK_MINUS = 35;  ///< updownlink- (Serial8 TX)
static constexpr uint8_t PIN_UPDOWNLINK_PLUS = 34;   ///< updownlink+ (Serial8 RX)
