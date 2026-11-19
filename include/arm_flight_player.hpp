#pragma once
#include <cstdint>
#include "hal/arm_hal.hpp"

/**
 * @file arm_flight_player.hpp
 * @brief Flug-Trajektorie des Roboterarms (Zustand ROBOT_ARM)
 *
 * Spielt die Tabelle aus arm_flight_trajectory.hpp ab (erzeugt aus der
 * Flug-CSV mit `tools/arm_trajectory.py flight`). Die Tabelle steht im Flash
 * der Firmware - die SD-Karte wird dafuer nicht gelesen.
 *
 * Unterschiede zum Bodentest (ArmTrajPlayer):
 *   - ABSOLUTE Gelenkwinkel, alle Gelenke der Tabelle (bis ARM_MAX_JOINTS).
 *   - Bei Problemen eines Gelenks (keine Meldung, Schleppfehler) wird NICHT
 *     abgebrochen: Die Bahn ist kollisionsfrei geplant, ein Abweichen davon
 *     (Stopp mitten im Raum, direkter Weg zur Verstaupose) nicht. Probleme
 *     werden gemeldet (Fehlermaske), ein haengendes Gelenk bekommt jede
 *     Sekunde ein neues RELEASE (falls es neu gestartet und gesperrt ist).
 *   - Warten statt Auseinanderlaufen: Haengt ein MELDENDES Gelenk mehr als
 *     TRACK_ERR_DEG hinter der Bahn, bleibt die Bahnzeit stehen (die anderen
 *     halten ihr Ziel) und das Gelenk holt mit MAX_VEL_DEG_S auf. So bleibt
 *     die Stellung des Arms auf der geplanten Bahn. Insgesamt hoechstens
 *     PAUSE_BUDGET_MS pro Flug, danach laeuft die Zeit ohne Warten weiter
 *     (ein klemmendes Gelenk darf nicht die ganze Bahn anhalten). Ein stummes
 *     Gelenk haelt nichts auf - ueber seine Lage weiss der OBC nichts.
 *   - Neustart des OBC: Fortsetzen am zuletzt gesicherten Punkt, nicht am
 *     Punkt der aktuellen Flugzeit (siehe resume()).
 *
 * Wann er startet, entscheidet System (Freigabe: Flug-Schalter ODER
 * Schwerelosigkeit, nur in ROBOT_ARM). Der Player selbst prueft nur die
 * Tabelle.
 *
 * ---------------------------------------------------------------------------
 * Ablauf
 * ---------------------------------------------------------------------------
 *   start()   -> RELEASE + langsam auf Punkt 0 (Startpose = Verstaupose)
 *   APPROACH  -> bis alle meldenden Gelenke naeher als START_TOL_DEG an Punkt 0
 *                sind, hoechstens APPROACH_TIMEOUT_MS. Danach geht es in jedem
 *                Fall los (Abweichung steht in max_err / Log).
 *   RUN       -> alle DT_MS je Gelenk SET_VELOCITY + MOVE_ABS auf den Punkt
 *                LEAD Schritte voraus (wie ArmTrajPlayer, am Gelenk erprobt).
 *   FINISH    -> letzter Punkt (Verstaupose), warten bis erreicht oder
 *                FINISH_TIMEOUT_MS.
 *   DONE      -> fertig, es wird nichts mehr gesendet.
 *
 *   resume()  -> RESUME_WAIT: bis alle Gelenke melden (hoechstens
 *                RESUME_WAIT_MS), dann den Tabellenpunkt suchen, der am besten
 *                zu den gemeldeten Winkeln passt (ab dem gesicherten Schritt,
 *                RESUME_SEARCH_STEPS weit), und von dort in RUN weiter.
 */

/// Eine Flug-Tabelle. System nimmt die eingebaute (arm_flight_trajectory.hpp),
/// der Host-Test eigene.
struct ArmFlightTable {
    bool           valid;
    uint32_t       dt_ms;
    uint8_t        joints;
    uint16_t       len;
    uint32_t       crc32;
    const uint8_t* nodes;    ///< [joints]
    const int32_t* urad;     ///< [len][joints], zeilenweise
};

/// Die eingebaute Tabelle aus arm_flight_trajectory.hpp.
const ArmFlightTable& builtinFlightTable();

class ArmFlightPlayer {
public:
    // -----------------------------------------------------------------------
    // Grenzen - muessen zu tools/arm_trajectory.py (FLIGHT_*) passen. Die
    // Tabelle wird beim Booten dagegen geprueft (verifyTable).
    // -----------------------------------------------------------------------
    static constexpr float    MAX_VEL_DEG_S       = 10.0f;
    static constexpr float    ACCEL_DEG_S2        = 4.0f;
    static constexpr uint32_t MAX_DURATION_MS     = 90000;
    static constexpr float    END_TOL_DEG         = 0.2f;   ///< Ende = Start
    static constexpr float    MIN_VEL_DEG_S       = 0.5f;
    static constexpr float    SLOW_VEL_DEG_S      = 5.0f;   ///< Anfahrt / letzter Punkt
    static constexpr uint8_t  LEAD                = 2;      ///< Zielvorlauf in Schritten

    static constexpr float    START_TOL_DEG       = 1.0f;
    static constexpr uint32_t APPROACH_TIMEOUT_MS = 5000;
    static constexpr float    DONE_TOL_DEG        = 0.5f;
    static constexpr uint32_t FINISH_TIMEOUT_MS   = 10000;
    static constexpr uint32_t STATE_TIMEOUT_MS    = 1500;   ///< Gelenk gilt als stumm
    /// Ab hier wartet die Bahn auf das Gelenk; haelt das TRACK_ERR_MS lang an,
    /// gilt es als Problem (Fehlermaske, RELEASE). Am Gelenk gemessen
    /// (Trajektorientest 17.11.2026): normal bis ~2,5 Grad.
    static constexpr float    TRACK_ERR_DEG       = 5.0f;
    static constexpr uint32_t TRACK_ERR_MS        = 1000;
    static constexpr uint32_t PAUSE_BUDGET_MS     = 5000;   ///< Warten insgesamt pro Flug
    static constexpr uint32_t RELEASE_RETRY_MS    = 1000;
    static constexpr uint32_t RESUME_WAIT_MS      = 2000;
    static constexpr uint16_t RESUME_SEARCH_STEPS = 30;     ///< 3 s ab dem gesicherten Punkt

    /// Werte gehen ueber die Leitung (SYS/ARM_FLIGHT) - nur anhaengen.
    enum class Phase : uint8_t {
        IDLE        = 0,   ///< nicht gestartet / abgebrochen
        APPROACH    = 1,
        RUN         = 2,
        FINISH      = 3,
        DONE        = 4,
        RESUME_WAIT = 5,
    };

    explicit ArmFlightPlayer(const ArmFlightTable& table) : t_(table) {}

    /// Tabelle pruefen (gueltig, CRC, Nodes, Geschwindigkeit, Dauer, Ende =
    /// Start). Einmal beim Booten, Ergebnis in tableOk(). Grund im Log.
    bool verifyTable();
    bool tableOk() const { return table_ok_; }
    const ArmFlightTable& table() const { return t_; }

    /// Neu starten (Punkt 0). false = Tabelle nicht ok oder TX-Puffer voll.
    bool start(ArmLink& arm, uint32_t now_ms);
    /// Nach OBC-Neustart fortsetzen. saved_step ist der zuletzt gesicherte
    /// Schritt, saved_valid false wenn keiner lesbar war (dann ganze Tabelle
    /// durchsuchen).
    bool resume(uint16_t saved_step, bool saved_valid, uint32_t now_ms);
    /// Jeden Loop aufrufen.
    void update(ArmLink& arm, uint32_t now_ms);
    /// Von aussen beenden (Rueckkehr an den Boden, Flugbeginn). Sendet nichts -
    /// der Aufrufer schickt STOP/INHIBIT.
    void cancel(const char* reason);

    bool active() const { return phase_ != Phase::IDLE && phase_ != Phase::DONE; }
    bool done()   const { return phase_ == Phase::DONE; }

    // Telemetrie
    Phase    phase()       const { return phase_; }
    uint16_t step()        const { return step_; }
    float    maxErrDeg()   const { return max_err_deg_; }
    uint8_t  faultSticky() const { return fault_sticky_; }  ///< Bit k = Spalte k hatte ein Problem
    uint8_t  faultLive()   const { return fault_live_; }    ///< Bit k = Spalte k hat gerade eins
    bool     resumed()     const { return resumed_; }
    bool     holding()     const { return holding_; }        ///< Bahn wartet gerade auf ein Gelenk
    uint32_t pauseUsedMs() const { return pause_used_ms_; }
    uint16_t txSkips()     const { return tx_skips_; }
    bool     takeChanged() { const bool c = changed_; changed_ = false; return c; }

private:
    const ArmFlightTable& t_;
    bool     table_ok_     = false;

    Phase    phase_        = Phase::IDLE;
    uint32_t phase_ms_     = 0;
    uint16_t base_step_    = 0;      ///< Schritt bei Beginn von RUN
    uint32_t progress_ms_  = 0;      ///< Bahnzeit seit Beginn von RUN (steht beim Warten)
    uint32_t last_update_ms_ = 0;
    uint32_t pause_used_ms_ = 0;
    bool     holding_      = false;
    uint8_t  lag_mask_     = 0;      ///< meldende Gelenke ueber TRACK_ERR_DEG
    int32_t  last_sent_    = -1;     ///< zuletzt gesendeter Schritt in RUN
    uint8_t  last_sent_lag_ = 0;     ///< lag_mask_ beim letzten Senden
    uint16_t step_         = 0;
    uint16_t saved_step_   = 0;
    bool     saved_valid_  = false;
    bool     resumed_      = false;
    bool     finish_sent_  = false;

    float    max_err_deg_  = 0.0f;
    uint8_t  fault_sticky_ = 0;
    uint8_t  fault_live_   = 0;
    uint32_t track_err_since_[ARM_MAX_JOINTS] = {};
    uint32_t last_release_ms_ = 0;
    uint16_t tx_skips_     = 0;
    uint32_t last_log_ms_  = 0;
    bool     changed_      = false;

    int32_t  target(int32_t step, uint8_t col) const;
    uint16_t last() const { return static_cast<uint16_t>(t_.len - 1); }
    void     setPhase(Phase p, uint32_t now_ms);
    /// Gelenk meldet frisch? (Signed-Vergleich, siehe ArmTrajPlayer::update.)
    static bool fresh(const ArmJoint* j, uint32_t now_ms);
    /// Fehlermaske und Schleppfehler gegen Schritt ref aktualisieren.
    void     monitor(ArmLink& arm, uint32_t now_ms, int32_t ref, bool track);
    /// Alle Gelenke auf Punkt step, Geschwindigkeit je Gelenk. false = nicht
    /// gesendet (TX-Puffer reicht nicht fuer ALLE Gelenke - nie halb senden).
    /// Gelenke in catchup fahren mit MAX_VEL_DEG_S (aufholen).
    bool     sendAll(ArmLink& arm, int32_t step, int32_t from_step, bool slow, uint8_t catchup = 0);
    void     enterRun(uint16_t step, uint32_t now_ms);
    /// Gelenke mit Problem jede RELEASE_RETRY_MS neu freigeben.
    void     retryRelease(ArmLink& arm, uint32_t now_ms);
    bool     allNear(ArmLink& arm, uint32_t now_ms, int32_t step, float tol_deg) const;
    uint16_t bestResumeStep(ArmLink& arm, uint32_t now_ms) const;
    void     logProgress(ArmLink& arm, uint32_t now_ms);
};
