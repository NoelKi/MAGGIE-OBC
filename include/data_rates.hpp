#pragma once

#include <cstdint>

/**
 * @file data_rates.hpp
 * @brief Datenraten fuer Downlink UND SD-Karte - alle an einer Stelle
 *
 * Jedes Frame wird einmal erzeugt und geht an zwei getrennte Ziele:
 *
 *   SD-Karte   bekommt ALLES, plus Ereignis-Frames (Zustand, SODS/LO/SOE) auf
 *              die Millisekunde genau, die nie ueber die UART gehen.
 *   Downlink   bekommt dieselben Frames, solange die UART mitkommt. Ist ihr
 *              Sendepuffer voll, faellt das Frame NUR fuer den Downlink weg -
 *              der Loop wartet nie auf die UART (siehe TelemetryDownlink).
 *
 * Datenmenge bei diesen Raten (20 Byte/Frame):
 *   IMU   100 Hz x 2 Frames   4000 B/s
 *   Kraft  80 Hz x 2 Sensoren 3200 B/s   (Rate gibt der HX711 vor, s.u.)
 *   Motor  20 Hz x 2 Motoren   800 B/s
 *   SYS     1 Hz x 3 Frames     60 B/s
 *   Summe                    ~8 KB/s  -> UART braucht >= 115200 Baud
 */
namespace rates {

/// IMU (Accel + Gyro). Der BMI088 laeuft intern mit 400 Hz und Tiefpass
/// (siehe imu_hal.cpp), damit jeder Lesezugriff einen frischen Wert sieht.
constexpr uint32_t IMU_INTERVAL_MS   = 10;     // 100 Hz

/// Motor-Position, PWM und Status beider Motoren.
constexpr uint32_t MOTOR_INTERVAL_MS = 50;     // 20 Hz

/// Zustand, Subsysteme, REXUS-Pegel, Uplink-Statistik.
constexpr uint32_t SYS_INTERVAL_MS   = 1000;   // 1 Hz

// Kraftsensoren: KEIN Intervall. Jede Wandlung geht sofort raus - die Rate
// bestimmt der RATE-Pin der HX711-Platinen (10 oder 80 Hz, Hardware).

/// Baudrate von Serial4 - gilt fuer Downlink UND Uplink (eine UART, eine
/// Baudrate). 38400 Baud tragen ~3,8 KB/s: Zustand und REXUS-Signale kommen
/// immer durch (Vorrang, siehe TelemetryDownlink), von den Sensordaten etwa
/// die Haelfte - vollstaendig stehen sie auf der SD-Karte.
///
/// MUSS auf allen Seiten gleich sein, sonst fehlen Downlink UND Telecommands:
///   - MAGGIE-SERVER .env: SERIAL_BAUD und TC_SERIAL_BAUD (im Bodenaufbau
///     derselbe FTDI-Adapter fuer beide Richtungen)
///   - GUI: Baudrate beim Verbinden des Downlinks
///   - RXSM/Test-Modul: dessen Experiment-Schnittstelle.
/// Ein Versuch mit 115200 (2026-10-26) lief am Aufbau NICHT.
constexpr uint32_t DOWNLINK_BAUDRATE = 38400;

}  // namespace rates
