#pragma once

#include <cstdint>

/**
 * @file data_rates.hpp
 * @brief Datenraten fuer Downlink UND SD-Karte - alle an einer Stelle
 *
 * Zwei getrennte Ziele:
 *
 *   SD-Karte   bekommt JEDEN Messwert direkt von der Quelle, unabhaengig vom
 *              Downlink (siehe sd_logger.hpp, CSV je Subsystem):
 *                IMU 100 Hz, Kraft jede Wandlung (~80 Hz je Sensor), Motoren
 *                SD_MOTOR_INTERVAL_MS, Arm JEDES Frame der RCU (RCU-IMU
 *                ~104 Hz, Gelenke, Spannung, Temperatur, Modul-IMU),
 *                Zustaende/Telecommands/Warnungen als Ereignisse, das
 *                komplette Log. Grob 15-25 KB/s Text.
 *   Downlink   drei Stufen (Nutzer, 19.11.2026), siehe TelemetryDownlink:
 *                1. STATUS  - alle SYS-Frames: Zustand, HDRM, Arm/Kameras/
 *                   Sensoren an, Diagnose. Duerfen den ganzen UART-Puffer
 *                   nutzen, gehen praktisch nie verloren.
 *                2. BEWEGUNG - ARM/JOINT (Gelenkwinkel): bis DL_MOTION_QUEUE_MS
 *                   Warteschlange, laesst immer Platz fuer STATUS.
 *                3. REST    - IMU, Kraft, Motoren, RCU-IMU, Arm-Spannung/
 *                   -Temperatur: nur bei kurzer Warteschlange
 *                   (DL_BULK_QUEUE_MS), und vorher auf DL_*_INTERVAL_MS
 *                   ausgeduennt, damit die Summe in die Leitung passt und
 *                   jede Art gleichmaessig ankommt statt zufaellig.
 *              Ist der UART-Puffer voll, faellt ein Frame NUR fuer den
 *              Downlink weg - der Loop wartet nie auf die UART.
 *
 * Downlink-Budget bei 38400 Baud (~3840 B/s, 20 Byte/Frame):
 *   1. STATUS    SYS/STATE 2 Hz + 9 Diagnose-Frames 1 Hz      ~220 B/s
 *   2. BEWEGUNG  Gelenke 7 x 5 Hz (nur neue Werte)            ~700 B/s
 *   3. REST      IMU 2 x 20 Hz                                 800 B/s
 *                Kraft 2 x 10 Hz                               400 B/s
 *                Motoren 2 x 10 Hz                             400 B/s
 *                RCU-IMU 2 x 2 Hz                               80 B/s
 *                Arm-Spannung/-Temperatur 7 x 2 x 1 Hz         280 B/s
 *   Summe                                                    ~2900 B/s (~75 %)
 *
 * HOEHERE BAUDRATE (finales Experiment): nur DOWNLINK_BAUDRATE aendern. Die
 * Intervalle der Stufe 3 und die Warteschlangen-Grenzen skalieren mit
 * (DL_SPEED), bis zur Rate der Sensoren selbst.
 */
namespace rates {

/// IMU (Accel + Gyro) lesen - SD-Karte bekommt jeden Wert. Der BMI088 laeuft
/// intern mit 100 Hz (Accel) bzw. 400 Hz (Gyro), siehe imu_hal.cpp.
constexpr uint32_t IMU_INTERVAL_MS   = 10;     // 100 Hz

/// Motoren fuer die SD-Karte (MOTOR.csv).
constexpr uint32_t SD_MOTOR_INTERVAL_MS = 10;  // 100 Hz

// Kraftsensoren: KEIN Leseintervall. Jede Wandlung geht auf die SD-Karte - die
// Rate bestimmt der RATE-Pin der HX711-Platinen (80 Hz, Hardware).

/// Baudrate von Serial4 - gilt fuer Downlink UND Uplink (eine UART, eine
/// Baudrate).
///
/// MUSS auf allen Seiten gleich sein, sonst fehlen Downlink UND Telecommands:
///   - MAGGIE-SERVER .env: SERIAL_BAUD und TC_SERIAL_BAUD (im Bodenaufbau
///     derselbe FTDI-Adapter fuer beide Richtungen)
///   - GUI: Baudrate beim Verbinden des Downlinks
///   - RXSM/Test-Modul: dessen Experiment-Schnittstelle.
/// Ein Versuch mit 115200 (2026-10-26) lief am Aufbau NICHT - der
/// Simulator schafft offenbar nicht mehr. Im Flug kann es mehr sein.
#ifndef MAGGIE_DOWNLINK_BAUDRATE
#define MAGGIE_DOWNLINK_BAUDRATE 38400   // nur tools/downlink_sim ueberschreibt das
#endif
constexpr uint32_t DOWNLINK_BAUDRATE = MAGGIE_DOWNLINK_BAUDRATE;

// ---------------------------------------------------------------------------
// Downlink (siehe Dateikopf). Alles abgeleitet aus DOWNLINK_BAUDRATE.
// ---------------------------------------------------------------------------

/// Bytes pro Sekunde auf der Leitung (8N1 = 10 Bit je Byte).
constexpr uint32_t DL_BYTES_PER_S = DOWNLINK_BAUDRATE / 10;

/// Vielfaches von 38400 Baud - die Stufe-3-Raten wachsen damit mit.
constexpr uint32_t DL_SPEED = DOWNLINK_BAUDRATE / 38400 > 0 ? DOWNLINK_BAUDRATE / 38400 : 1;

/// Stufe-3-Intervall: Basis bei 38400 Baud, geteilt durch DL_SPEED, aber
/// nie schneller als die Quelle selbst liefert.
constexpr uint32_t dlInterval(uint32_t base_ms, uint32_t source_ms) {
    return base_ms / DL_SPEED > source_ms ? base_ms / DL_SPEED : source_ms;
}

/// Stufe 1 - STATUS
constexpr uint32_t STATUS_INTERVAL_MS = 500;    ///< SYS/STATE (Zustand, HDRM, an/aus), 2 Hz
constexpr uint32_t SYS_INTERVAL_MS    = 1000;   ///< uebrige SYS-Frames (Diagnose), 1 Hz

/// Stufe 2 - BEWEGUNG: Gelenkzustand je Gelenk, nur wenn ein neuer kam.
constexpr uint32_t ARM_JOINT_INTERVAL_MS = 200;   // 5 Hz

/// Stufe 3 - REST
constexpr uint32_t DL_IMU_INTERVAL_MS   = dlInterval(50, IMU_INTERVAL_MS);   // 20 Hz bei 38400
constexpr uint32_t DL_FORCE_INTERVAL_MS = dlInterval(100, 12);               // 10 Hz je Sensor
constexpr uint32_t DL_MOTOR_INTERVAL_MS = dlInterval(100, 10);               // 10 Hz je Motor
constexpr uint32_t ARM_IMU_INTERVAL_MS  = dlInterval(500, 10);               // RCU-IMU 2 Hz
constexpr uint32_t ARM_POWER_INTERVAL_MS = 1000;   ///< Spannung + Temperatur je Gelenk, 1 Hz

/// Warteschlangen-Grenzen als Sendezeit: so viele ms duerfen in der UART
/// stehen, bevor die Stufe ihr Frame verwirft. STATUS darf immer den ganzen
/// Puffer nutzen. Bei 38400 Baud: Rest ~230 Byte, Bewegung ~307 Byte - ein
/// Status-Frame wartet also hoechstens ~60 ms hinter Sensordaten.
constexpr uint32_t DL_BULK_QUEUE_MS   = 60;
constexpr uint32_t DL_MOTION_QUEUE_MS = 80;
constexpr int DL_BULK_QUEUE_MAX   = static_cast<int>(DL_BYTES_PER_S * DL_BULK_QUEUE_MS / 1000);
constexpr int DL_MOTION_QUEUE_MAX = static_cast<int>(DL_BYTES_PER_S * DL_MOTION_QUEUE_MS / 1000);
/// So viel vom UART-Puffer bleibt immer fuer STATUS frei (ein voller
/// 1-s-Schub SYS-Frames).
constexpr int DL_STATUS_RESERVE = 220;

// ---------------------------------------------------------------------------
// Weitere UARTs
// ---------------------------------------------------------------------------

/// Baudrate der UART zur RCU (Serial1). UNBESTAETIGT: Die ICD legt sie nicht
/// fest (bei LISA-ODISEE laeuft die UART ueber USB). Am Arm pruefen - laeuft
/// nichts, zeigt die Link-Statistik (SYS/ARM_LINK) frames_ok = 0, bei falscher
/// Baudrate meist steigende frames_bad.
constexpr uint32_t ARM_BAUDRATE = 115200;

/// Baudrate der Kamera-UART (Serial5, RunCam Device Protocol). An allen vier
/// Split 4 V2 bestaetigt (19.11.2026).
constexpr uint32_t CAM_BAUDRATE = 115200;

}  // namespace rates
