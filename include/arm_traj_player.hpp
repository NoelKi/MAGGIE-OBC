#pragma once
#include <cstdint>
#include "hal/arm_hal.hpp"

/**
 * @file arm_traj_player.hpp
 * @brief Trajektorientest an hoechstens ZWEI Gelenken (Bodentest, nur TEST)
 *
 * Spielt die Tabelle aus arm_test_trajectory.hpp (erzeugt aus einer ROS-CSV
 * mit tools/arm_trajectory.py) synchron an 1 oder 2 Gelenken ab. Spalte k der
 * Tabelle faehrt die k-te Node aus dem Telecommand. Die Tabelle enthaelt
 * Winkel RELATIV zum Start: Basis ist je Gelenk der Winkel, den es beim Start
 * meldet. So laeuft die Fahrt nur um den aktuellen Stand herum, nie auf
 * absolute Winkel aus der Simulation.
 *
 * Freigabe: Trajektorientest mit hoechstens zwei Gelenken vom Team am
 * 15.11.2026 fuer den Testaufbau freigegeben (CLAUDE.md, Regel 2). Mehr als
 * MAX_JOINTS Gelenke nimmt der Player nicht an - nicht erhoehen, ohne das
 * Team zu fragen.
 *
 * ---------------------------------------------------------------------------
 * Ablauf
 * ---------------------------------------------------------------------------
 *   start()  -> RELEASE, langsames MOVE_ABS auf den aktuellen Winkel (je Gelenk)
 *   SETTLE   -> 500 ms halten. Bewegt sich ein Gelenk dabei um mehr als
 *               SETTLE_TOL, passt der Bezug von MOVE_ABS nicht zum gemeldeten
 *               Winkel -> Abbruch, bevor irgendetwas gefahren ist.
 *   RUN      -> alle DT_MS (100 ms) je Gelenk: SET_VELOCITY + MOVE_ABS auf den
 *               Punkt LEAD Schritte voraus. Das Ziel liegt immer etwas vor dem
 *               Gelenk, MOVE_ABS aendert nur das Ziel - das Gelenk faehrt so
 *               ohne Halt mit der Geschwindigkeit des Abschnitts durch.
 *   FINISH   -> letztes Ziel = Startwinkel, warten bis alle Gelenke dort sind.
 *
 * Abbruch (STOP + INHIBIT an ALLE beteiligten Gelenke): Schleppfehler zu
 * gross, keine JOINT_STATE mehr, TX-Puffer voll, Rueckfahrt dauert zu lange.
 * Von aussen: cancel() bei ARM_STOP/ARM_INHIBIT, TEST verlassen, ABORT.
 *
 * Grenzen (doppelt, auch das Skript prueft sie): jedes Ziel liegt innerhalb
 * +-MAX_DEG um den Startwinkel des Gelenks, Geschwindigkeit <= MAX_VEL_DEG_S.
 */
class ArmTrajPlayer {
public:
    /// Hoechstens so viele Gelenke gleichzeitig (Freigabe des Teams).
    static constexpr uint8_t  MAX_JOINTS       = 2;
    /// Groesste Auslenkung um den Startwinkel - gleich ARM_JOG_MAX_DEG.
    static constexpr float    MAX_DEG          = 30.0f;
    /// Hoechste Geschwindigkeit, die der Player je setzt.
    static constexpr float    MAX_VEL_DEG_S    = 20.0f;
    /// Kleinste Geschwindigkeit (0 waere fuer die RCU unzulaessig/undefiniert).
    static constexpr float    MIN_VEL_DEG_S    = 0.5f;
    /// Beschleunigung fuer alle Abschnitte.
    static constexpr float    ACCEL_DEG_S2     = 40.0f;
    /// Zielvorlauf in Tabellenschritten (2 x 100 ms).
    static constexpr uint8_t  LEAD             = 2;
    /// Langsame Fahrt beim Halten vor dem Start und auf den Startwinkel zurueck.
    static constexpr float    SLOW_VEL_DEG_S   = 5.0f;

    static constexpr uint32_t SETTLE_MS        = 500;
    static constexpr float    SETTLE_TOL_DEG   = 2.0f;
    /// Schleppfehler, ab dem nach TRACK_ERR_MS abgebrochen wird.
    static constexpr float    TRACK_ERR_DEG    = 8.0f;
    static constexpr uint32_t TRACK_ERR_MS     = 1000;
    /// So alt darf der letzte JOINT_STATE waehrend der Fahrt hoechstens sein.
    static constexpr uint32_t STATE_TIMEOUT_MS = 1500;
    /// Am Ziel, wenn naeher als das am Startwinkel.
    static constexpr float    DONE_TOL_DEG     = 0.5f;

    /**
     * Startet die Tabelle. nodes[k] faehrt Spalte k; count muss gleich der
     * Spaltenzahl der Tabelle sein. false = nicht gestartet (Grund im Log).
     */
    bool start(ArmLink& arm, const uint8_t* nodes, uint8_t count, uint32_t now_ms);

    /// Jeden Loop aufrufen.
    void update(ArmLink& arm, uint32_t now_ms);

    /// Von aussen beenden, ohne selbst etwas zu senden (der Aufrufer schickt
    /// STOP/INHIBIT ohnehin). Gibt true zurueck, wenn eine Fahrt lief.
    bool cancel(const char* reason);

    bool active() const { return phase_ != Phase::IDLE; }
    /// Ist diese Node an der laufenden Fahrt beteiligt?
    bool uses(uint8_t node) const;

private:
    enum class Phase : uint8_t { IDLE, SETTLE, RUN, FINISH };

    struct Joint {
        uint8_t  node      = 0;
        int32_t  base_urad = 0;
        uint32_t track_err_since_ms = 0;
    };

    Phase    phase_       = Phase::IDLE;
    Joint    joints_[MAX_JOINTS];
    uint8_t  count_       = 0;
    uint32_t phase_ms_    = 0;     ///< Beginn der aktuellen Phase
    int32_t  last_step_   = -1;    ///< zuletzt gesendeter Tabellenschritt
    uint32_t last_log_ms_ = 0;

    void abort(ArmLink& arm, const char* reason, uint8_t node);
    bool sendTarget(ArmLink& arm, const Joint& j, int32_t offset_urad, float vel_deg_s);
    static bool checkTable();
};
