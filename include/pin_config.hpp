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
 * Source: Teensy 4.1 Pinout Diagram (MAGGIE OBC v1.6)
 */

// ===========================================================================
// UART / Serial Communication
// ===========================================================================
// ARM Communication (UART1)
static constexpr uint8_t PIN_ARM_TX = 0;       ///< UART1 TX
static constexpr uint8_t PIN_ARM_RX = 1;       ///< UART1 RX

// ===========================================================================
// Force Sensors 1
// ===========================================================================
// Sensor 1 (Force X, Y, Z) - Analog Input
static constexpr uint8_t PIN_FORCE_X_1 = 2;   ///< Analog Input
static constexpr uint8_t PIN_FORCE_Y_1 = 3;   ///< Analog Input
static constexpr uint8_t PIN_FORCE_Z_1 = 4;   ///< Analog Input

// ===========================================================================
// Cameras
// ===========================================================================
// 4x RunCam Split 4 (115200 8N1) an EINEM UART (Serial2). TX und RX werden
// gemeinsam über einen 4:1-Mux auf die jeweilige Kamera geschaltet.
// Signalnamen aus Sicht der KAMERA: CAM..._TX = Ausgang der Kamera.
static constexpr uint8_t PIN_CAM1_TX = 7;      ///< Teensy RX2  <- Mux <- Kamera TX
static constexpr uint8_t PIN_CAM_MAIN_RX = 8;  ///< Teensy TX2  -> Mux -> Kamera RX

// Kanalwahl des Mux: Kanal = (CAMDIR2 << 1) | CAMDIR1, also 0..3 = Kamera 1..4.
//
// ACHTUNG - Doppelbelegung: Pin 19 und 22 sind in diesem Header zusätzlich als
// PIN_M3_B bzw. PIN_M2_B vergeben. Motor 2 und 3 werden aktuell nirgends
// instanziiert, der Konflikt ist also noch theoretisch. Beim Finalisieren der
// Pinbelegung auflösen. Werte stammen aus
// docs/teensyPins/MAGGIE-OCB-PIN-BELEGUNG.txt (CAMDIR1 = 19, CAMDIR2 = 22).
static constexpr uint8_t PIN_CAM_MUX_A = 19;   ///< CAMDIR1, LSB der Kanalwahl
static constexpr uint8_t PIN_CAM_MUX_B = 22;   ///< CAMDIR2, MSB der Kanalwahl

// ===========================================================================
// Force CLK 2
// ===========================================================================
static constexpr uint8_t PIN_FORCE_CLK_2 = 9;   ///< Analog Input
static constexpr uint8_t PIN_CHIP_SELECT_FORCE = 10;   ///< Chip Select (Digital OUT)

// ===========================================================================
// SPI Bus
// ===========================================================================
// Standard Teensy 4.1 SPI1 Pins
static constexpr uint8_t PIN_SPI1_MOSI = 11;    ///< MOSI
static constexpr uint8_t PIN_SPI1_MISO = 12;    ///< MISO

// ===========================================================================
// Cameras
// ===========================================================================
// Zweiter Kamera-UART (Serial6). Seit dem Mux-Aufbau nicht mehr belegt - alle
// vier Kameras hängen an Serial2. Konstanten bleiben als Reserve stehen.
static constexpr uint8_t PIN_CAM_BACKUP_RX = 24;  ///< Teensy TX6, ungenutzt
static constexpr uint8_t PIN_CAM3_TX = 25;        ///< Teensy RX6, ungenutzt

static constexpr uint8_t PIN_CS_TEMP = 26;

// ===========================================================================
// Force CLK 1
// ===========================================================================
static constexpr uint8_t PIN_FORCE_CLK_1 = 27;   ///< Analog Input

// ===========================================================================
// Force Sensors 2
// ===========================================================================
// Sensor 2 (Force X, Y, Z) - Analog Input
static constexpr uint8_t PIN_FORCE_X_2 = 30;   ///< Analog Input
static constexpr uint8_t PIN_FORCE_Y_2 = 31;   ///< Analog Input
static constexpr uint8_t PIN_FORCE_Z_2 = 32;   ///< Analog Input

// ===========================================================================
// Motor PWM Outputs (DRV8871 Driver)
// ===========================================================================
// Motor 2 (Channels A+B)
static constexpr uint8_t PIN_M2_B = 22;        ///< Motor 2 Channel B
static constexpr uint8_t PIN_M2_A = 23;        ///< Motor 2 Channel A

// Motor 3 (Channels A+B)
static constexpr uint8_t PIN_M3_A = 18;        ///< Motor 3 Channel A
static constexpr uint8_t PIN_M3_B = 19;        ///< Motor 3 Channel B

// Motor 1 (Channels A+B)
static constexpr uint8_t PIN_M1_B = 40;        ///< Motor 1 Channel B
static constexpr uint8_t PIN_M1_A = 41;        ///< Motor 1 Channel A
// SCK
static constexpr uint8_t PIN_SCK = 13;        ///< SCK

// Motor 1 Quadratur-Encoder (A/B) - Closed-Loop Positionsregelung.
// Verbaut: Pololu enc03d (0J12461) am Getriebemotor.
//
// Pin 0/1 sind in der Belegungstabelle die ARM-UART (Serial1). Der Roboterarm
// wird in dieser Firmware nirgends instanziiert - es gibt kein Serial1.begin(),
// nur die ungenutzten Aliase ARM_TX_PIN/ARM_RX_PIN in sensor_hal.hpp. Die Pins
// sind damit reines GPIO und frei. Auf dem Teensy 4.1 ist jeder Digitalpin
// interruptfähig, die Encoder-Bibliothek arbeitet hier also normal.
//
// ACHTUNG: Sobald die ARM-Kommunikation dazukommt, kollidiert sie hier - dann
// muss der Encoder umziehen (frei waeren dann z.B. 18+20).
static constexpr uint8_t PIN_M1_ENC_A = 0;     ///< Motor 1 Encoder Channel A
static constexpr uint8_t PIN_M1_ENC_B = 1;     ///< Motor 1 Encoder Channel B

// ===========================================================================
// REXUS Signals
// ===========================================================================
// L0_t lag urspruenglich auf Pin 40 - dort haengt jetzt PIN_M1_B. REXUSHAL::init()
// laeuft in System::init() NACH dem Motor und wuerde den Pin mit INPUT_PULLDOWN
// zurueckkonfigurieren, der Motorkanal waere damit tot. Deshalb auf Pin 21
// ausgewichen: frei, und ohnehin ohne PWM-Timer - ein reiner Digitaleingang
// verschwendet dort also keinen der knappen PWM-faehigen Pins.
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
// Sensors
// ===========================================================================
// Chip Select Pins
static constexpr uint8_t PIN_CS_ACCEL = 37;    ///< Chip Select Accelerometer
static constexpr uint8_t PIN_CS_GYRO = 36;     ///< Chip Select Gyroscope

// ===========================================================================
// Up/Down-link
// ===========================================================================
static constexpr uint8_t PIN_UPDOWNLINK_MINUS = 35;  ///< updownlink-
static constexpr uint8_t PIN_UPDOWNLINK_PLUS = 34;  ///< updownlink+

// ===========================================================================
// HX711 Aliases (Sensor 1 - Purchased Scale)
// ===========================================================================
static constexpr uint8_t PIN_HX711_DOUT = PIN_FORCE_X_1;  ///< HX711 Data Out (mapped to Sensor 1)
static constexpr uint8_t PIN_HX711_SCK = PIN_FORCE_Y_1;   ///< HX711 Serial Clock (mapped to Sensor 1)
