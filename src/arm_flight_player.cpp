#include "arm_flight_player.hpp"
#include "obc_log.hpp"
#include "arm_flight_trajectory.hpp"
#include <Arduino.h>
#include <math.h>

static_assert(ARM_FLIGHT_JOINTS >= 1 && ARM_FLIGHT_JOINTS <= ARM_MAX_JOINTS,
              "Flug-Tabelle hat mehr Gelenke als der OBC verwaltet (ARM_MAX_JOINTS)");
static_assert(ARM_FLIGHT_LEN >= 2, "Flug-Tabelle braucht mindestens 2 Punkte");
static_assert(ARM_MAX_JOINTS <= 8, "Fehlermasken sind 8 Bit breit");

const ArmFlightTable& builtinFlightTable() {
    static const ArmFlightTable table = {
        ARM_FLIGHT_VALID, ARM_FLIGHT_DT_MS, ARM_FLIGHT_JOINTS, ARM_FLIGHT_LEN,
        ARM_FLIGHT_CRC32, ARM_FLIGHT_NODES, &ARM_FLIGHT_URAD[0][0],
    };
    return table;
}

// 1 Grad in Mikroradiant (pi / 180 * 1e6) - die RCU rechnet in urad.
static constexpr float URAD_PER_DEG = 17453.2925f;

static float uradToDeg(int32_t urad) { return static_cast<float>(urad) / URAD_PER_DEG; }
static int32_t degToUrad(float deg) { return static_cast<int32_t>(lroundf(deg * URAD_PER_DEG)); }
static int32_t absI32(int32_t v) { return v < 0 ? -v : v; }
static bool elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t span_ms) {
    return static_cast<int32_t>(now_ms - since_ms) >= static_cast<int32_t>(span_ms);
}

// Frame-Laengen (Header 4 + Payload + CRC 4), siehe arm_hal.hpp.
static constexpr int FRAME_COMMAND  = 8;
static constexpr int FRAME_VELOCITY = 16;
static constexpr int FRAME_MOVE     = 12;

// ---------------------------------------------------------------------------
// Tabelle
// ---------------------------------------------------------------------------

int32_t ArmFlightPlayer::target(int32_t step, uint8_t col) const {
    if (step < 0) step = 0;
    if (step > last()) step = last();
    return t_.urad[static_cast<uint32_t>(step) * t_.joints + col];
}

bool ArmFlightPlayer::verifyTable() {
    table_ok_ = false;
    if (!t_.valid) {
        Log.println("WARN  [ArmFlight]: Keine Flug-Trajektorie eingebaut (Platzhalter) - "
                       "der Arm faehrt im Flug NICHT.");
        return false;
    }
    if (t_.dt_ms != 100 || t_.len < 2 || t_.joints < 1 || t_.joints > ARM_MAX_JOINTS) {
        Log.printf("WARN  [ArmFlight]: Tabelle unbrauchbar (Takt %lu ms, %u Punkte, %u Gelenke) - "
                      "der Arm faehrt im Flug NICHT.\n",
                      static_cast<unsigned long>(t_.dt_ms), t_.len, t_.joints);
        return false;
    }
    const uint32_t duration_ms = static_cast<uint32_t>(t_.len - 1) * t_.dt_ms;
    if (duration_ms > MAX_DURATION_MS) {
        Log.printf("WARN  [ArmFlight]: Tabelle dauert %lu ms, erlaubt %lu - der Arm faehrt im Flug NICHT.\n",
                      static_cast<unsigned long>(duration_ms), static_cast<unsigned long>(MAX_DURATION_MS));
        return false;
    }
    for (uint8_t k = 0; k < t_.joints; k++) {
        const uint8_t node = t_.nodes[k];
        bool dup = false;
        for (uint8_t m = 0; m < k; m++) dup |= t_.nodes[m] == node;
        if (node < 1 || node > ARM_MAX_JOINTS || dup) {
            Log.printf("WARN  [ArmFlight]: Node %u in Spalte %u ungueltig oder doppelt - "
                          "der Arm faehrt im Flug NICHT.\n", node, k + 1);
            return false;
        }
    }

    const size_t bytes = static_cast<size_t>(t_.len) * t_.joints * sizeof(int32_t);
    const uint32_t crc = ArmLink::crc32(reinterpret_cast<const uint8_t*>(t_.urad), bytes);
    if (crc != t_.crc32) {
        Log.printf("WARN  [ArmFlight]: CRC der Tabelle 0x%08lX statt 0x%08lX (von Hand veraendert?) - "
                      "der Arm faehrt im Flug NICHT.\n",
                      static_cast<unsigned long>(crc), static_cast<unsigned long>(t_.crc32));
        return false;
    }

    // Dieselben Grenzen wie das Skript, unabhaengig davon noch einmal. Kleine
    // Reserve fuer die Rundung auf ganze urad.
    const int32_t max_step = degToUrad(MAX_VEL_DEG_S * t_.dt_ms / 1000.0f) + 50;
    const int32_t end_tol  = degToUrad(END_TOL_DEG);
    for (uint8_t k = 0; k < t_.joints; k++) {
        if (absI32(target(last(), k) - target(0, k)) > end_tol) {
            Log.printf("WARN  [ArmFlight]: Spalte %u endet nicht in der Startpose - "
                          "der Arm faehrt im Flug NICHT.\n", k + 1);
            return false;
        }
        for (uint16_t i = 1; i < t_.len; i++) {
            if (absI32(target(i, k) - target(i - 1, k)) > max_step) {
                Log.printf("WARN  [ArmFlight]: Spalte %u Schritt %u schneller als %.0f Grad/s - "
                              "der Arm faehrt im Flug NICHT.\n", k + 1, i, MAX_VEL_DEG_S);
                return false;
            }
        }
    }

    table_ok_ = true;
    Log.printf("INFO  [ArmFlight]: Flug-Trajektorie OK - %u Punkte (%.1f s), %u Gelenke, CRC 0x%08lX.\n",
                  t_.len, duration_ms / 1000.0f, t_.joints, static_cast<unsigned long>(crc));
    for (uint8_t k = 0; k < t_.joints; k++) {
        Log.printf("INFO  [ArmFlight]:   Spalte %u -> Node %u, Startpose %+.2f Grad.\n",
                      k + 1, t_.nodes[k], uradToDeg(target(0, k)));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Steuerung
// ---------------------------------------------------------------------------

void ArmFlightPlayer::setPhase(Phase p, uint32_t now_ms) {
    phase_    = p;
    phase_ms_ = now_ms;
    changed_  = true;
}

bool ArmFlightPlayer::fresh(const ArmJoint* j, uint32_t now_ms) {
    return j && j->seen &&
           static_cast<int32_t>(now_ms - j->state_ms) <= static_cast<int32_t>(STATE_TIMEOUT_MS);
}

bool ArmFlightPlayer::start(ArmLink& arm, uint32_t now_ms) {
    if (!table_ok_ || active() || done()) return false;
    if (arm.txFree() < t_.joints * (FRAME_COMMAND + FRAME_VELOCITY + FRAME_MOVE)) {
        tx_skips_++;
        return false;   // naechster Loop
    }

    resumed_      = false;
    max_err_deg_  = 0.0f;
    fault_sticky_ = 0;
    fault_live_   = 0;
    for (auto& s : track_err_since_) s = 0;

    for (uint8_t k = 0; k < t_.joints; k++) arm.sendCommand(t_.nodes[k], ArmMsg::RELEASE);
    sendAll(arm, 0, 0, /*slow=*/true);

    step_        = 0;
    last_log_ms_ = now_ms;
    setPhase(Phase::APPROACH, now_ms);
    Log.printf("INFO  [ArmFlight]: Start - Gelenke freigegeben, fahren langsam auf die Startpose "
                  "(hoechstens %lu ms).\n", static_cast<unsigned long>(APPROACH_TIMEOUT_MS));
    return true;
}

bool ArmFlightPlayer::resume(uint16_t saved_step, bool saved_valid, uint32_t now_ms) {
    if (!table_ok_ || active() || done()) return false;
    resumed_     = true;
    saved_step_  = saved_step > last() ? last() : saved_step;
    saved_valid_ = saved_valid;
    for (auto& s : track_err_since_) s = 0;
    last_log_ms_ = now_ms;
    setPhase(Phase::RESUME_WAIT, now_ms);
    if (saved_valid) {
        Log.printf("INFO  [ArmFlight]: Fortsetzen nach Neustart, gesicherter Schritt %u - "
                      "warte auf Gelenkmeldungen.\n", saved_step_);
    } else {
        Log.println("WARN  [ArmFlight]: Fortsetzen nach Neustart OHNE gesicherten Schritt - "
                       "suche den passenden Punkt in der ganzen Tabelle.");
    }
    return true;
}

void ArmFlightPlayer::cancel(const char* reason) {
    if (!active()) return;
    Log.printf("WARN  [ArmFlight]: abgebrochen (%s) bei Schritt %u.\n", reason, step_);
    phase_   = Phase::IDLE;
    changed_ = true;
}

bool ArmFlightPlayer::sendAll(ArmLink& arm, int32_t step, int32_t from_step, bool slow,
                              uint8_t catchup) {
    // Alle Gelenke oder keins: Bekommt nur ein Teil ein neues Ziel, laufen
    // die Gelenke auseinander. Dann lieber im naechsten Loop alle zusammen.
    if (arm.txFree() < t_.joints * (FRAME_VELOCITY + FRAME_MOVE)) {
        tx_skips_++;
        return false;
    }
    const float span_s = static_cast<float>(step - from_step) * t_.dt_ms / 1000.0f;
    for (uint8_t k = 0; k < t_.joints; k++) {
        const int32_t tgt = target(step, k);
        float vel = SLOW_VEL_DEG_S;
        if (!slow) {
            // Mittlere Geschwindigkeit bis zum Ziel, 10 % Reserve gegen Nachlaufen
            // (wie ArmTrajPlayer).
            vel = span_s > 0.0f ? fabsf(uradToDeg(tgt - target(from_step, k))) / span_s * 1.1f
                                : MIN_VEL_DEG_S;
        }
        if (catchup & (1u << k)) vel = MAX_VEL_DEG_S;
        if (vel > MAX_VEL_DEG_S) vel = MAX_VEL_DEG_S;
        if (vel < MIN_VEL_DEG_S) vel = MIN_VEL_DEG_S;
        arm.sendSetVelocity(t_.nodes[k], vel, ACCEL_DEG_S2);
        arm.sendMoveAbs(t_.nodes[k], tgt);
    }
    return true;
}

void ArmFlightPlayer::monitor(ArmLink& arm, uint32_t now_ms, int32_t ref, bool track) {
    uint8_t live = 0;
    uint8_t lag  = 0;
    for (uint8_t k = 0; k < t_.joints; k++) {
        const uint8_t bit = static_cast<uint8_t>(1u << k);
        const ArmJoint* j = arm.joint(t_.nodes[k]);
        if (!fresh(j, now_ms)) {
            live |= bit;
            track_err_since_[k] = 0;
            continue;
        }
        if (!track) {
            track_err_since_[k] = 0;
            continue;
        }
        const float err = fabsf(uradToDeg(j->angle_urad - target(ref, k)));
        if (err > max_err_deg_) max_err_deg_ = err;
        if (err > TRACK_ERR_DEG) {
            lag |= bit;
            if (track_err_since_[k] == 0) track_err_since_[k] = now_ms ? now_ms : 1;
            if (elapsed(now_ms, track_err_since_[k], TRACK_ERR_MS)) live |= bit;
        } else {
            track_err_since_[k] = 0;
        }
    }

    const uint8_t fresh_faults = live & static_cast<uint8_t>(~fault_live_);
    for (uint8_t k = 0; k < t_.joints; k++) {
        if (!(fresh_faults & (1u << k))) continue;
        const ArmJoint* j = arm.joint(t_.nodes[k]);
        Log.printf("WARN  [ArmFlight]: Node %u %s - Bahn laeuft weiter.\n", t_.nodes[k],
                      fresh(j, now_ms) ? "folgt nicht (Schleppfehler)" : "meldet sich nicht");
    }
    if (live != fault_live_) changed_ = true;
    fault_live_    = live;
    fault_sticky_ |= live;
    lag_mask_      = lag;
}

void ArmFlightPlayer::retryRelease(ArmLink& arm, uint32_t now_ms) {
    // Nur Gelenke, die melden, aber nicht folgen - etwa nach einem eigenen
    // Neustart wieder gesperrt. Ein stummes Gelenk kann trotzdem richtig
    // fahren; ihm mitten in der Bewegung RELEASE zu schicken, bringt nichts.
    if (!fault_live_ || !elapsed(now_ms, last_release_ms_, RELEASE_RETRY_MS)) return;
    last_release_ms_ = now_ms;
    for (uint8_t k = 0; k < t_.joints; k++) {
        if (!(fault_live_ & (1u << k))) continue;
        if (!fresh(arm.joint(t_.nodes[k]), now_ms)) continue;
        if (arm.txFree() < FRAME_COMMAND) return;
        arm.sendCommand(t_.nodes[k], ArmMsg::RELEASE);
        // Ob RELEASE eine laufende Fahrt beendet, legt die ICD nicht fest -
        // das Ziel deshalb gleich danach noch einmal schicken.
        last_sent_ = -1;
    }
}

bool ArmFlightPlayer::allNear(ArmLink& arm, uint32_t now_ms, int32_t step, float tol_deg) const {
    // Nur meldende Gelenke zaehlen - ein stummes soll den Ablauf nicht
    // aufhalten (dafuer gibt es die Zeitgrenzen). Meldet keins: nicht fertig.
    bool any = false;
    for (uint8_t k = 0; k < t_.joints; k++) {
        const ArmJoint* j = arm.joint(t_.nodes[k]);
        if (!fresh(j, now_ms)) continue;
        any = true;
        if (fabsf(uradToDeg(j->angle_urad - target(step, k))) > tol_deg) return false;
    }
    return any;
}

uint16_t ArmFlightPlayer::bestResumeStep(ArmLink& arm, uint32_t now_ms) const {
    const uint16_t lo = saved_valid_ ? saved_step_ : 0;
    uint32_t hi32 = saved_valid_ ? static_cast<uint32_t>(saved_step_) + RESUME_SEARCH_STEPS : last();
    const uint16_t hi = static_cast<uint16_t>(hi32 > last() ? last() : hi32);

    uint16_t best = lo;
    float best_cost = -1.0f;
    for (uint16_t s = lo; s <= hi; s++) {
        float cost = -1.0f;   // groesste Abweichung ueber die meldenden Gelenke
        for (uint8_t k = 0; k < t_.joints; k++) {
            const ArmJoint* j = arm.joint(t_.nodes[k]);
            if (!fresh(j, now_ms)) continue;
            const float e = fabsf(uradToDeg(j->angle_urad - target(s, k)));
            if (e > cost) cost = e;
        }
        if (cost < 0.0f) break;   // kein Gelenk meldet - beim gesicherten Schritt bleiben
        if (best_cost < 0.0f || cost < best_cost) {
            best_cost = cost;
            best = s;
        }
        if (s == hi) break;       // uint16-Ueberlauf bei hi = 65535 vermeiden
    }
    if (best_cost < 0.0f) {
        Log.printf("WARN  [ArmFlight]: Kein Gelenk meldet - setze bei Schritt %u fort.\n", best);
    } else {
        Log.printf("INFO  [ArmFlight]: Setze bei Schritt %u fort (gesucht %u..%u), "
                      "groesste Abweichung %.2f Grad.\n", best, lo, hi, best_cost);
    }
    return best;
}

void ArmFlightPlayer::enterRun(uint16_t step, uint32_t now_ms) {
    base_step_      = step;
    step_           = step;
    progress_ms_    = 0;
    last_update_ms_ = now_ms;
    holding_        = false;
    lag_mask_       = 0;
    last_sent_      = -1;
    last_sent_lag_  = 0;
    finish_sent_    = false;
    setPhase(step >= last() ? Phase::FINISH : Phase::RUN, now_ms);
}

void ArmFlightPlayer::update(ArmLink& arm, uint32_t now_ms) {
    switch (phase_) {
        case Phase::IDLE:
        case Phase::DONE:
            return;

        case Phase::RESUME_WAIT: {
            bool all = true;
            for (uint8_t k = 0; k < t_.joints; k++) all &= fresh(arm.joint(t_.nodes[k]), now_ms);
            if (!all && !elapsed(now_ms, phase_ms_, RESUME_WAIT_MS)) return;
            if (arm.txFree() < t_.joints * FRAME_COMMAND) {
                tx_skips_++;
                return;
            }
            const uint16_t s = bestResumeStep(arm, now_ms);
            // Die Gelenke halten das letzte Ziel (READY). RELEASE dort ist
            // harmlos (am Gelenk erprobt: ARM_JOG schickt es vor jeder Fahrt)
            // und hilft, falls eins zwischendurch gesperrt wurde.
            for (uint8_t k = 0; k < t_.joints; k++) arm.sendCommand(t_.nodes[k], ArmMsg::RELEASE);
            enterRun(s, now_ms);
            return;
        }

        case Phase::APPROACH: {
            monitor(arm, now_ms, 0, /*track=*/false);
            const bool near = allNear(arm, now_ms, 0, START_TOL_DEG);
            if (!near && !elapsed(now_ms, phase_ms_, APPROACH_TIMEOUT_MS)) return;
            if (!near) {
                Log.println("WARN  [ArmFlight]: Startpose nicht von allen Gelenken erreicht - "
                               "Bahn startet trotzdem.");
            }
            for (uint8_t k = 0; k < t_.joints; k++) {
                const ArmJoint* j = arm.joint(t_.nodes[k]);
                if (!fresh(j, now_ms)) continue;
                const float e = fabsf(uradToDeg(j->angle_urad - target(0, k)));
                if (e > max_err_deg_) max_err_deg_ = e;
                Log.printf("INFO  [ArmFlight]:   Node %u: %.2f Grad von der Startpose.\n", t_.nodes[k], e);
            }
            enterRun(0, now_ms);
            Log.println("INFO  [ArmFlight]: Bahn laeuft.");
            return;
        }

        case Phase::RUN: {
            // Bahnzeit weiterzaehlen - oder warten, solange ein meldendes
            // Gelenk hinterherhaengt (lag_mask_ aus dem letzten Loop).
            // Nie "negativ" (siehe ArmTrajPlayer since()): ein Sprung um ~4 Mrd. ms
            // wuerde Bahnzeit und Pausenbudget auf einen Schlag verbrauchen.
            const int32_t dt_signed = static_cast<int32_t>(now_ms - last_update_ms_);
            const uint32_t dt = dt_signed > 0 ? static_cast<uint32_t>(dt_signed) : 0;
            last_update_ms_ = now_ms;
            const bool hold = lag_mask_ && pause_used_ms_ < PAUSE_BUDGET_MS;
            if (hold) {
                pause_used_ms_ += dt;
            } else {
                progress_ms_ += dt;
            }
            if (hold != holding_) {
                holding_ = hold;
                changed_ = true;
                if (hold) {
                    Log.printf("WARN  [ArmFlight]: Gelenk(e) 0x%02X haengen nach - Bahn wartet "
                                  "(bisher %lu von %lu ms).\n", lag_mask_,
                                  static_cast<unsigned long>(pause_used_ms_),
                                  static_cast<unsigned long>(PAUSE_BUDGET_MS));
                } else if (pause_used_ms_ >= PAUSE_BUDGET_MS && lag_mask_) {
                    Log.println("WARN  [ArmFlight]: Wartezeit aufgebraucht - Bahn laeuft ohne Warten weiter.");
                } else {
                    Log.println("INFO  [ArmFlight]: aufgeholt, Bahn laeuft weiter.");
                }
            }

            const uint32_t s32 = base_step_ + progress_ms_ / t_.dt_ms;
            const uint16_t s   = static_cast<uint16_t>(s32 > last() ? last() : s32);
            step_ = s;
            monitor(arm, now_ms, s, /*track=*/true);
            retryRelease(arm, now_ms);
            logProgress(arm, now_ms);
            if (s >= last()) {
                finish_sent_ = false;
                setPhase(Phase::FINISH, now_ms);
                Log.println("INFO  [ArmFlight]: Tabelle durch, Gelenke fahren in die Verstaupose.");
                return;
            }
            // Neues Ziel bei neuem Schritt - und sofort, wenn ein Gelenk neu
            // nachhaengt (es soll mit voller Geschwindigkeit aufholen).
            if (static_cast<int32_t>(s) != last_sent_ || lag_mask_ != last_sent_lag_) {
                const int32_t tgt = s + LEAD < last() ? s + LEAD : last();
                if (sendAll(arm, tgt, s, /*slow=*/false, lag_mask_)) {
                    last_sent_     = s;
                    last_sent_lag_ = lag_mask_;
                }
            }
            return;
        }

        case Phase::FINISH: {
            monitor(arm, now_ms, last(), /*track=*/false);
            if (!finish_sent_) {
                finish_sent_ = sendAll(arm, last(), last(), /*slow=*/true);
                return;
            }
            const bool near = allNear(arm, now_ms, last(), DONE_TOL_DEG);
            if (!near && !elapsed(now_ms, phase_ms_, FINISH_TIMEOUT_MS)) return;
            Log.printf("%s [ArmFlight]: Fertig - %s. Groesster Schleppfehler %.2f Grad, "
                          "Fehlermaske 0x%02X.\n",
                          near ? "INFO " : "WARN ",
                          near ? "alle meldenden Gelenke in der Verstaupose"
                               : "Verstaupose NICHT von allen Gelenken bestaetigt",
                          max_err_deg_, fault_sticky_);
            setPhase(Phase::DONE, now_ms);
            return;
        }
    }
}

void ArmFlightPlayer::logProgress(ArmLink& arm, uint32_t now_ms) {
    if (!elapsed(now_ms, last_log_ms_, 1000)) return;
    last_log_ms_ = now_ms;
    Log.printf("INFO  [ArmFlight]: Schritt %u/%u", step_, last());
    for (uint8_t k = 0; k < t_.joints; k++) {
        const ArmJoint* j = arm.joint(t_.nodes[k]);
        if (fresh(j, now_ms)) {
            Log.printf(" | N%u %+.1f/%+.1f", t_.nodes[k],
                          uradToDeg(target(step_, k)), uradToDeg(j->angle_urad));
        } else {
            Log.printf(" | N%u stumm", t_.nodes[k]);
        }
    }
    Log.println();
}
