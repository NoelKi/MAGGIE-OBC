#include "arm_traj_player.hpp"
#include "arm_test_trajectory.hpp"
#include <Arduino.h>
#include <math.h>

// 1 Grad in Mikroradiant (pi / 180 * 1e6) - die RCU rechnet in urad.
static constexpr float URAD_PER_DEG = 17453.2925f;

static int32_t degToUrad(float deg) {
    return static_cast<int32_t>(lroundf(deg * URAD_PER_DEG));
}

static float uradToDeg(int32_t urad) {
    return static_cast<float>(urad) / URAD_PER_DEG;
}

static int32_t offsetAt(int32_t step) {
    if (step < 0) step = 0;
    if (step >= static_cast<int32_t>(ARM_TRAJ_LEN)) step = ARM_TRAJ_LEN - 1;
    return ARM_TRAJ_OFFSET_URAD[step];
}

bool ArmTrajPlayer::checkTable() {
    // Das Skript prueft dasselbe - hier noch einmal, falls jemand die
    // Tabelle von Hand aendert. Lieber nicht starten als falsch fahren.
    if (ARM_TRAJ_LEN < 2 || ARM_TRAJ_DT_MS != 100) {
        Serial.println("WARN  [ArmTraj]: Tabelle leer oder Takt nicht 100 ms - nicht gestartet.");
        return false;
    }
    if (ARM_TRAJ_OFFSET_URAD[0] != 0 || ARM_TRAJ_OFFSET_URAD[ARM_TRAJ_LEN - 1] != 0) {
        Serial.println("WARN  [ArmTraj]: Tabelle beginnt/endet nicht bei 0 (Start = Ende) - nicht gestartet.");
        return false;
    }
    const int32_t max_urad = degToUrad(MAX_DEG);
    const int32_t max_step = degToUrad(MAX_VEL_DEG_S * ARM_TRAJ_DT_MS / 1000.0f);
    for (uint16_t i = 0; i < ARM_TRAJ_LEN; i++) {
        const int32_t o = ARM_TRAJ_OFFSET_URAD[i];
        if (o > max_urad || o < -max_urad) {
            Serial.printf("WARN  [ArmTraj]: Punkt %u (%.2f Grad) ausserhalb +-%.0f Grad - nicht gestartet.\n",
                          i, uradToDeg(o), MAX_DEG);
            return false;
        }
        if (i > 0) {
            const int32_t d = o - ARM_TRAJ_OFFSET_URAD[i - 1];
            if (d > max_step || d < -max_step) {
                Serial.printf("WARN  [ArmTraj]: Schritt %u zu schnell (%.1f Grad/s > %.0f) - nicht gestartet.\n",
                              i, uradToDeg(d) * 1000.0f / ARM_TRAJ_DT_MS, MAX_VEL_DEG_S);
                return false;
            }
        }
    }
    return true;
}

bool ArmTrajPlayer::start(ArmLink& arm, uint8_t node, uint32_t now_ms) {
    if (active()) {
        Serial.printf("WARN  [ArmTraj]: laeuft bereits an Gelenk %u - nicht gestartet.\n", node_);
        return false;
    }
    if (!checkTable()) return false;

    ArmJoint* j = arm.joint(node);
    if (!j || !j->seen || !arm.linkAlive(now_ms)) {
        Serial.printf("WARN  [ArmTraj]: Gelenk %u meldet keinen Winkel (RCU-Link %s) - nicht gestartet.\n",
                      node, arm.linkAlive(now_ms) ? "ok" : "tot");
        return false;
    }

    node_      = node;
    base_urad_ = j->angle_urad;
    last_step_ = -1;
    track_err_since_ms_ = 0;

    // Einmal freigeben, dann auf den aktuellen Winkel "fahren" (= halten).
    if (!arm.sendCommand(node_, ArmMsg::RELEASE) || !sendTarget(arm, 0, SLOW_VEL_DEG_S)) {
        Serial.println("WARN  [ArmTraj]: TX-Puffer voll - nicht gestartet.");
        arm.sendCommand(node_, ArmMsg::STOP);
        return false;
    }

    phase_    = Phase::SETTLE;
    phase_ms_ = now_ms;
    last_log_ms_ = now_ms;
    Serial.printf("INFO  [ArmTraj]: Start Gelenk %u bei %.2f Grad, %u Punkte (%.1f s), "
                  "Bereich +-%.0f Grad, max %.0f Grad/s.\n",
                  node_, uradToDeg(base_urad_), ARM_TRAJ_LEN,
                  ARM_TRAJ_LEN * ARM_TRAJ_DT_MS / 1000.0f, MAX_DEG, MAX_VEL_DEG_S);
    return true;
}

bool ArmTrajPlayer::sendTarget(ArmLink& arm, int32_t offset_urad, float vel_deg_s) {
    // Grenze unabhaengig von der Tabelle: nie weiter als MAX_DEG vom Start.
    const int32_t lim = degToUrad(MAX_DEG);
    if (offset_urad >  lim) offset_urad =  lim;
    if (offset_urad < -lim) offset_urad = -lim;
    if (vel_deg_s > MAX_VEL_DEG_S) vel_deg_s = MAX_VEL_DEG_S;
    if (vel_deg_s < MIN_VEL_DEG_S) vel_deg_s = MIN_VEL_DEG_S;

    const uint32_t v = static_cast<uint32_t>(degToUrad(vel_deg_s));
    const uint32_t a = static_cast<uint32_t>(degToUrad(ACCEL_DEG_S2));
    return arm.sendSetVelocity(node_, v, a) &&
           arm.sendMoveAbs(node_, base_urad_ + offset_urad);
}

void ArmTrajPlayer::update(ArmLink& arm, uint32_t now_ms) {
    if (!active()) return;

    ArmJoint* j = arm.joint(node_);
    if (!j) { abort(arm, "Gelenk unbekannt"); return; }
    const float actual_deg = uradToDeg(j->angle_urad - base_urad_);   // relativ zum Start

    if (phase_ == Phase::SETTLE) {
        if (fabsf(actual_deg) > SETTLE_TOL_DEG) {
            abort(arm, "Gelenk bewegt sich beim Halten - Bezug von MOVE_ABS passt nicht zum gemeldeten Winkel");
            return;
        }
        if (now_ms - phase_ms_ >= SETTLE_MS) {
            phase_    = Phase::RUN;
            phase_ms_ = now_ms;
            Serial.printf("INFO  [ArmTraj]: Gelenk %u haelt, Trajektorie laeuft.\n", node_);
        }
        return;
    }

    // Ab hier faehrt das Gelenk: ohne frische Rueckmeldung nicht weiter.
    if (now_ms - j->state_ms > STATE_TIMEOUT_MS) {
        abort(arm, "keine JOINT_STATE mehr von der RCU");
        return;
    }

    if (phase_ == Phase::RUN) {
        const int32_t step = static_cast<int32_t>((now_ms - phase_ms_) / ARM_TRAJ_DT_MS);
        const int32_t last = ARM_TRAJ_LEN - 1;

        // Schleppfehler gegen den Sollwert JETZT (nicht gegen das vorausliegende Ziel).
        const float err = fabsf(actual_deg - uradToDeg(offsetAt(step)));
        if (err > TRACK_ERR_DEG) {
            if (track_err_since_ms_ == 0) track_err_since_ms_ = now_ms;
            if (now_ms - track_err_since_ms_ >= TRACK_ERR_MS) {
                abort(arm, "Schleppfehler zu gross");
                return;
            }
        } else {
            track_err_since_ms_ = 0;
        }

        if (step >= last) {
            // Ende der Tabelle: zurueck auf den Startwinkel (Tabelle endet bei 0,
            // das Ziel ist also derselbe Punkt - nur ohne Vorlauf).
            if (!sendTarget(arm, 0, SLOW_VEL_DEG_S)) { abort(arm, "TX-Puffer voll"); return; }
            phase_    = Phase::FINISH;
            phase_ms_ = now_ms;
            Serial.printf("INFO  [ArmTraj]: Tabelle durch, Gelenk %u faehrt auf den Startwinkel.\n", node_);
            return;
        }

        if (step != last_step_) {
            last_step_ = step;
            const int32_t tgt = step + LEAD < last ? step + LEAD : last;
            const int32_t span_urad = offsetAt(tgt) - offsetAt(step);
            const float span_s = (tgt - step) * ARM_TRAJ_DT_MS / 1000.0f;
            // Mittlere Geschwindigkeit bis zum Ziel, 10 % Reserve gegen Nachlaufen.
            const float vel = span_s > 0.0f ? fabsf(uradToDeg(span_urad)) / span_s * 1.1f : MIN_VEL_DEG_S;
            if (!sendTarget(arm, offsetAt(tgt), vel)) { abort(arm, "TX-Puffer voll"); return; }
        }

        if (now_ms - last_log_ms_ >= 1000) {
            last_log_ms_ = now_ms;
            Serial.printf("INFO  [ArmTraj]: t=%.1f s soll %+.2f ist %+.2f Grad (Fehler %.2f)\n",
                          (now_ms - phase_ms_) / 1000.0f, uradToDeg(offsetAt(step)), actual_deg, err);
        }
        return;
    }

    // FINISH: auf den Startwinkel warten.
    if (fabsf(actual_deg) <= DONE_TOL_DEG) {
        Serial.printf("INFO  [ArmTraj]: fertig - Gelenk %u wieder am Startwinkel (%+.2f Grad Abweichung).\n",
                      node_, actual_deg);
        phase_ = Phase::IDLE;
        return;
    }
    const uint32_t timeout_ms = static_cast<uint32_t>(MAX_DEG / SLOW_VEL_DEG_S * 1000.0f) + 3000;
    if (now_ms - phase_ms_ > timeout_ms) {
        abort(arm, "Startwinkel nicht erreicht");
    }
}

void ArmTrajPlayer::abort(ArmLink& arm, const char* reason) {
    Serial.printf("WARN  [ArmTraj]: Abbruch Gelenk %u - %s. STOP + INHIBIT.\n", node_, reason);
    arm.sendCommand(node_, ArmMsg::STOP);
    arm.sendCommand(node_, ArmMsg::INHIBIT);
    phase_ = Phase::IDLE;
}

bool ArmTrajPlayer::cancel(const char* reason) {
    if (!active()) return false;
    Serial.printf("INFO  [ArmTraj]: beendet an Gelenk %u - %s.\n", node_, reason);
    phase_ = Phase::IDLE;
    return true;
}
