#include "arm_traj_player.hpp"
#include "arm_test_trajectory.hpp"
#include <Arduino.h>
#include <math.h>

static_assert(ARM_TRAJ_JOINTS >= 1 && ARM_TRAJ_JOINTS <= ArmTrajPlayer::MAX_JOINTS,
              "Tabelle hat mehr Gelenke als der Player abspielen darf");

// 1 Grad in Mikroradiant (pi / 180 * 1e6) - die RCU rechnet in urad.
static constexpr float URAD_PER_DEG = 17453.2925f;

static int32_t degToUrad(float deg) {
    return static_cast<int32_t>(lroundf(deg * URAD_PER_DEG));
}

static float uradToDeg(int32_t urad) {
    return static_cast<float>(urad) / URAD_PER_DEG;
}

/// Sollwinkel (relativ) von Spalte col im Schritt step, Schritt begrenzt.
static int32_t offsetAt(int32_t step, uint8_t col) {
    if (step < 0) step = 0;
    if (step >= static_cast<int32_t>(ARM_TRAJ_LEN)) step = ARM_TRAJ_LEN - 1;
    return ARM_TRAJ_OFFSET_URAD[step][col];
}

bool ArmTrajPlayer::checkTable() {
    // Das Skript prueft dasselbe - hier noch einmal, falls jemand die
    // Tabelle von Hand aendert. Lieber nicht starten als falsch fahren.
    if (ARM_TRAJ_LEN < 2 || ARM_TRAJ_DT_MS != 100) {
        Serial.println("WARN  [ArmTraj]: Tabelle leer oder Takt nicht 100 ms - nicht gestartet.");
        return false;
    }
    const int32_t max_urad = degToUrad(MAX_DEG);
    const int32_t max_step = degToUrad(MAX_VEL_DEG_S * ARM_TRAJ_DT_MS / 1000.0f);
    for (uint8_t c = 0; c < ARM_TRAJ_JOINTS; c++) {
        if (offsetAt(0, c) != 0 || offsetAt(ARM_TRAJ_LEN - 1, c) != 0) {
            Serial.printf("WARN  [ArmTraj]: Spalte %u beginnt/endet nicht bei 0 (Start = Ende) - "
                          "nicht gestartet.\n", c + 1);
            return false;
        }
        for (uint16_t i = 0; i < ARM_TRAJ_LEN; i++) {
            const int32_t o = offsetAt(i, c);
            if (o > max_urad || o < -max_urad) {
                Serial.printf("WARN  [ArmTraj]: Spalte %u Punkt %u (%.2f Grad) ausserhalb +-%.0f Grad - "
                              "nicht gestartet.\n", c + 1, i, uradToDeg(o), MAX_DEG);
                return false;
            }
            if (i > 0) {
                const int32_t d = o - offsetAt(i - 1, c);
                if (d > max_step || d < -max_step) {
                    Serial.printf("WARN  [ArmTraj]: Spalte %u Schritt %u zu schnell (%.1f Grad/s > %.0f) - "
                                  "nicht gestartet.\n", c + 1, i,
                                  uradToDeg(d) * 1000.0f / ARM_TRAJ_DT_MS, MAX_VEL_DEG_S);
                    return false;
                }
            }
        }
    }
    return true;
}

bool ArmTrajPlayer::uses(uint8_t node) const {
    if (!active()) return false;
    for (uint8_t k = 0; k < count_; k++) {
        if (joints_[k].node == node) return true;
    }
    return false;
}

bool ArmTrajPlayer::start(ArmLink& arm, const uint8_t* nodes, uint8_t count, uint32_t now_ms) {
    if (active()) {
        // Laufende Fahrt nicht anfassen - auch nicht ihren Status.
        Serial.println("WARN  [ArmTraj]: laeuft bereits - nicht gestartet.");
        return false;
    }
    if (runs_ < 0xFF) runs_++;
    step_ = 0;
    max_err_deg_ = 0.0f;

    if (count != ARM_TRAJ_JOINTS) {
        Serial.printf("WARN  [ArmTraj]: %u Gelenk(e) angegeben, Tabelle hat %u Spalte(n) - nicht gestartet.\n",
                      count, ARM_TRAJ_JOINTS);
        return refuse();
    }
    if (!checkTable()) return refuse();
    if (!arm.linkAlive(now_ms)) {
        Serial.println("WARN  [ArmTraj]: RCU-Link tot - nicht gestartet.");
        return refuse();
    }

    for (uint8_t k = 0; k < count; k++) {
        ArmJoint* j = arm.joint(nodes[k]);
        if (!j || !j->seen) {
            Serial.printf("WARN  [ArmTraj]: Gelenk %u meldet keinen Winkel - nicht gestartet.\n", nodes[k]);
            return refuse();
        }
        for (uint8_t m = 0; m < k; m++) {
            if (nodes[m] == nodes[k]) {
                Serial.printf("WARN  [ArmTraj]: Gelenk %u doppelt angegeben - nicht gestartet.\n", nodes[k]);
                return refuse();
            }
        }
        joints_[k].node      = nodes[k];
        joints_[k].base_urad = j->angle_urad;
        joints_[k].track_err_since_ms = 0;
    }
    count_     = count;
    last_step_ = -1;

    // Einmal freigeben, dann auf den aktuellen Winkel "fahren" (= halten).
    for (uint8_t k = 0; k < count_; k++) {
        if (!arm.sendCommand(joints_[k].node, ArmMsg::RELEASE) ||
            !sendTarget(arm, joints_[k], 0, SLOW_VEL_DEG_S)) {
            Serial.println("WARN  [ArmTraj]: TX-Puffer voll - nicht gestartet.");
            for (uint8_t m = 0; m < count_; m++) arm.sendCommand(joints_[m].node, ArmMsg::STOP);
            result_ = Result::TX_FULL;
            result_node_ = joints_[k].node;
            changed_ = true;
            return false;
        }
    }

    result_      = Result::RUNNING;
    result_node_ = 0;
    setPhase(Phase::SETTLE);
    phase_ms_    = now_ms;
    last_log_ms_ = now_ms;
    for (uint8_t k = 0; k < count_; k++) {
        Serial.printf("INFO  [ArmTraj]: Spalte %u -> Gelenk %u, Start bei %.2f Grad.\n",
                      k + 1, joints_[k].node, uradToDeg(joints_[k].base_urad));
    }
    Serial.printf("INFO  [ArmTraj]: Start, %u Punkte (%.1f s), Bereich +-%.0f Grad, max %.0f Grad/s.\n",
                  ARM_TRAJ_LEN, ARM_TRAJ_LEN * ARM_TRAJ_DT_MS / 1000.0f, MAX_DEG, MAX_VEL_DEG_S);
    return true;
}

bool ArmTrajPlayer::sendTarget(ArmLink& arm, const Joint& j, int32_t offset_urad, float vel_deg_s) {
    // Grenze unabhaengig von der Tabelle: nie weiter als MAX_DEG vom Start.
    const int32_t lim = degToUrad(MAX_DEG);
    if (offset_urad >  lim) offset_urad =  lim;
    if (offset_urad < -lim) offset_urad = -lim;
    float v = vel_deg_s;
    if (v > MAX_VEL_DEG_S) v = MAX_VEL_DEG_S;
    if (v < MIN_VEL_DEG_S) v = MIN_VEL_DEG_S;

    return arm.sendSetVelocity(j.node, v, ACCEL_DEG_S2) &&
           arm.sendMoveAbs(j.node, j.base_urad + offset_urad);
}

void ArmTrajPlayer::update(ArmLink& arm, uint32_t now_ms) {
    if (!active()) return;

    // Istwinkel relativ zum Start, je Gelenk.
    float actual[MAX_JOINTS] = {};
    for (uint8_t k = 0; k < count_; k++) {
        ArmJoint* j = arm.joint(joints_[k].node);
        if (!j) { abort(arm, "Gelenk unbekannt", joints_[k].node, Result::NO_STATE); return; }
        actual[k] = uradToDeg(j->angle_urad - joints_[k].base_urad);
        // Ab RUN faehrt das Gelenk: ohne frische Rueckmeldung nicht weiter.
        // Ausnahme: in FINISH schon am Startwinkel - ein stehendes Gelenk
        // meldet sich womoeglich nicht mehr, waehrend das andere noch faehrt.
        const bool parked = phase_ == Phase::FINISH && fabsf(actual[k]) <= DONE_TOL_DEG;
        // Vorzeichenbehaftet vergleichen: now_ms ist der Loop-Anfang, poll()
        // stempelt state_ms aber erst danach mit millis(). Ein Frame aus
        // demselben Loop ist dann "juenger als jetzt" - unsigned waere die
        // Differenz ~4e9 und loeste faelschlich NO_STATE aus (17.11.2026:
        // zwei Abbrueche, obwohl die RCU lueckenlos alle 200 ms meldete).
        if (phase_ != Phase::SETTLE && !parked &&
            static_cast<int32_t>(now_ms - j->state_ms) > static_cast<int32_t>(STATE_TIMEOUT_MS)) {
            abort(arm, "keine JOINT_STATE mehr von der RCU", joints_[k].node, Result::NO_STATE);
            return;
        }
    }

    if (phase_ == Phase::SETTLE) {
        for (uint8_t k = 0; k < count_; k++) {
            if (fabsf(actual[k]) > max_err_deg_) max_err_deg_ = fabsf(actual[k]);
            if (fabsf(actual[k]) > SETTLE_TOL_DEG) {
                abort(arm, "bewegt sich beim Halten - Bezug von MOVE_ABS passt nicht zum gemeldeten Winkel",
                      joints_[k].node, Result::SETTLE_MOVED);
                return;
            }
        }
        if (now_ms - phase_ms_ >= SETTLE_MS) {
            setPhase(Phase::RUN);
            phase_ms_ = now_ms;
            Serial.println("INFO  [ArmTraj]: Gelenke halten, Trajektorie laeuft.");
        }
        return;
    }

    if (phase_ == Phase::RUN) {
        const int32_t step = static_cast<int32_t>((now_ms - phase_ms_) / ARM_TRAJ_DT_MS);
        const int32_t last = ARM_TRAJ_LEN - 1;
        step_ = static_cast<uint16_t>(step < last ? step : last);

        // Schleppfehler gegen den Sollwert JETZT (nicht gegen das vorausliegende Ziel).
        float err[MAX_JOINTS] = {};
        for (uint8_t k = 0; k < count_; k++) {
            err[k] = fabsf(actual[k] - uradToDeg(offsetAt(step, k)));
            if (err[k] > max_err_deg_) max_err_deg_ = err[k];
            Joint& j = joints_[k];
            if (err[k] > TRACK_ERR_DEG) {
                if (j.track_err_since_ms == 0) j.track_err_since_ms = now_ms;
                if (now_ms - j.track_err_since_ms >= TRACK_ERR_MS) {
                    abort(arm, "Schleppfehler zu gross", j.node, Result::TRACK_ERROR);
                    return;
                }
            } else {
                j.track_err_since_ms = 0;
            }
        }

        if (step >= last) {
            // Ende der Tabelle: zurueck auf den Startwinkel (Tabelle endet bei 0,
            // das Ziel ist also derselbe Punkt - nur ohne Vorlauf).
            for (uint8_t k = 0; k < count_; k++) {
                if (!sendTarget(arm, joints_[k], 0, SLOW_VEL_DEG_S)) {
                    abort(arm, "TX-Puffer voll", joints_[k].node, Result::TX_FULL);
                    return;
                }
            }
            setPhase(Phase::FINISH);
            phase_ms_ = now_ms;
            Serial.println("INFO  [ArmTraj]: Tabelle durch, Gelenke fahren auf den Startwinkel.");
            return;
        }

        if (step != last_step_) {
            last_step_ = step;
            const int32_t tgt = step + LEAD < last ? step + LEAD : last;
            const float span_s = (tgt - step) * ARM_TRAJ_DT_MS / 1000.0f;
            // Alle Gelenke im selben Takt direkt hintereinander: bei 115200 Baud
            // liegen ihre Frames < 3 ms auseinander.
            for (uint8_t k = 0; k < count_; k++) {
                const int32_t span_urad = offsetAt(tgt, k) - offsetAt(step, k);
                // Mittlere Geschwindigkeit bis zum Ziel, 10 % Reserve gegen Nachlaufen.
                const float vel = span_s > 0.0f ? fabsf(uradToDeg(span_urad)) / span_s * 1.1f : MIN_VEL_DEG_S;
                if (!sendTarget(arm, joints_[k], offsetAt(tgt, k), vel)) {
                    abort(arm, "TX-Puffer voll", joints_[k].node, Result::TX_FULL);
                    return;
                }
            }
        }

        if (now_ms - last_log_ms_ >= 1000) {
            last_log_ms_ = now_ms;
            Serial.printf("INFO  [ArmTraj]: t=%.1f s", (now_ms - phase_ms_) / 1000.0f);
            for (uint8_t k = 0; k < count_; k++) {
                Serial.printf(" | G%u soll %+.2f ist %+.2f (Fehler %.2f)", joints_[k].node,
                              uradToDeg(offsetAt(step, k)), actual[k], err[k]);
            }
            Serial.println();
        }
        return;
    }

    // FINISH: auf den Startwinkel warten - alle Gelenke.
    bool all_done = true;
    for (uint8_t k = 0; k < count_; k++) {
        if (fabsf(actual[k]) > DONE_TOL_DEG) all_done = false;
    }
    if (all_done) {
        Serial.print("INFO  [ArmTraj]: fertig - Gelenke wieder am Startwinkel (Abweichung");
        for (uint8_t k = 0; k < count_; k++) {
            Serial.printf(" G%u %+.2f", joints_[k].node, actual[k]);
        }
        Serial.println(" Grad).");
        finish(Result::DONE, 0);
        return;
    }
    // Weitester Rueckweg + Anfahren/Abbremsen mit ACCEL + Reserve.
    const uint32_t timeout_ms = static_cast<uint32_t>(
        (MAX_DEG / SLOW_VEL_DEG_S + 2.0f * SLOW_VEL_DEG_S / ACCEL_DEG_S2) * 1000.0f) + 3000;
    if (now_ms - phase_ms_ > timeout_ms) {
        abort(arm, "Startwinkel nicht erreicht", 0, Result::FINISH_TIMEOUT);
    }
}

void ArmTrajPlayer::abort(ArmLink& arm, const char* reason, uint8_t node, Result r) {
    if (node) Serial.printf("WARN  [ArmTraj]: Abbruch (Gelenk %u: %s). STOP + INHIBIT.\n", node, reason);
    else      Serial.printf("WARN  [ArmTraj]: Abbruch (%s). STOP + INHIBIT.\n", reason);
    // Erst alle anhalten, dann alle sperren - STOP ist das Dringende.
    for (uint8_t k = 0; k < count_; k++) arm.sendCommand(joints_[k].node, ArmMsg::STOP);
    for (uint8_t k = 0; k < count_; k++) arm.sendCommand(joints_[k].node, ArmMsg::INHIBIT);
    finish(r, node);
}

bool ArmTrajPlayer::cancel(const char* reason) {
    if (!active()) return false;
    Serial.printf("INFO  [ArmTraj]: beendet - %s.\n", reason);
    finish(Result::CANCELLED, 0);
    return true;
}
