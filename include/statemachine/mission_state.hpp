#pragma once

#include <cstdint>

/**
 * @file mission_state.hpp
 * @brief MAGGIE REXUS Experiment State Machine - Zustände & Konstanten
 */

enum class MissionState : uint8_t {
    PRE_LAUNCH,      // S0  - Idle, Selbsttests, SODS aus
    ARMED,           // S1  - Datalogger ein, Aktoren safe
    ASCENT,          // S2  - Boost, Vibration, Sensoren loggen
    MG_DETECT,       // S3a - warte auf erste Quelle (SOE | IMU)
    MG_CONFIRM,      // S3b - Cross-Check, warte auf zweite Quelle
    WAIT_FFU,        // S4  - Timer ab t_µg (FFU-Abwurf abwarten)
    HDRMS_OPEN,      // S5  - Aktuator (Motoren) löst aus
    ARM_DEPLOY,      // S6  - Arm: home -> pre-approach
    APPROACH_T1,     // S7  - Open-loop, Trajektorie T1
    SUCCESS_T1,      // S8  - Erfolg Target 1, Wegezelle + Timer
    APPROACH_T2,     // S10 - Open-loop, Trajektorie T2
    SUCCESS_T2,      // S11 - Erfolg Target 2
    ARM_STOW,        // S13 - Arm in HDRM-Konfiguration
    SUCCESS_STOW,    // S11 - Erfolg Stow, Wegezelle + Timer (Nummerierung so im Original-Diagramm)
    HDRM_CLOSE,      // S14 - Mechanismus schließt um Arm
    NET_DEPLOY,      // S15 - Redundantes Sicherungsnetz
    SAFE,            // S16 - Telemetrie weiter, Aktoren aus
    ABORT,           // S_E - Arm stop, HDRM zu wenn möglich, Netz immer auslösen
};

// Quelle für Mikrogravitations-Detektion
enum class MgSource : uint8_t {
    NONE,
    SOE_ONLY,
    IMU_ONLY,
    BOTH,       // präzisester Fall
    DEGRADED,   // Warnflag, Mission läuft weiter
};

// Konfiguration/Konstanten
namespace MissionConfig {
    // --- Microgravity-Detektion (SOE x IMU gekoppelt) ---
    static constexpr uint32_t T_MG_EARLIEST_MS = 60000;   // frühester MG-Detect ab LO
    static constexpr uint32_t T_MG_ABORT_MS    = 120000;  // kein MG -> Abort, ab LO
    static constexpr uint32_t T_XCHECK_MS      = 5000;    // Cross-Check-Fenster SOE<->IMU
    static constexpr float    IMU_G_THR        = 0.05f;   // Microgravity-Schwelle [g], |a_imu| < IMU_G_THR

    // TODO: IMU_t_thr nicht definiert
    static constexpr uint32_t IMU_T_THR_MS = 0;

    // --- Experiment-Sequenz (relativ zu t_µg) ---
    static constexpr uint32_t T_FFU_MS      = 10000;   // Wartezeit ab t_µg bis HDRMS_OPEN
    static constexpr uint32_t T_HARD_CUT_MS = 100000;  // t_µg + 100s -> erzwungener Übergang nach ARM_STOW (noch nicht verwendet, siehe TODO-Zustände)

    // --- Docking ---
    static constexpr float    F_THRESH_N = 0.2f;   // Kontakt-Erkennung Wegezelle (noch nicht verwendet)
    static constexpr float    F_MAX_N    = 5.0f;   // Force-Limit -> Abort
    static constexpr uint32_t T_HOLD_MS  = 3000;   // Halte-Zeit im DOCK (noch nicht verwendet)
}
