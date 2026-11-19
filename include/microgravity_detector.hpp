#pragma once

#include <cstdint>
#include <math.h>

/**
 * @file microgravity_detector.hpp
 * @brief Erkennt Schwerelosigkeit aus einer Beschleunigungsmessung
 *
 * Freigabe der Flug-Trajektorie des Roboterarms (siehe ArmFlightPlayer):
 * Am Boden misst jede IMU ~1 g, der Arm faehrt dort also nie von selbst los -
 * auch nicht, wenn ein Bodentest die REXUS-Signale bis ROBOT_ARM durchspielt.
 *
 * Schwerelos = Betrag der Beschleunigung unter THRESHOLD_G, ohne Unterbrechung
 * fuer HOLD_MS. Schutz gegen einen defekten Sensor, der 0 liefert:
 *   - ein exakter Nullvektor zaehlt als ungueltig (setzt zurueck),
 *   - die Werte muessen sich im Fenster mindestens einmal aendern (ein
 *     eingefrorener Sensor rauscht nicht),
 *   - eine Luecke von mehr als MAX_GAP_MS zwischen zwei Werten setzt zurueck.
 *
 * Schwelle: Nach dem Despin dreht REXUS kaum noch. Bei 0,2 Hz Restdrehung und
 * 0,15 m Abstand zur Achse sind das ~0,02 g - deutlich unter 0,1 g.
 */
class MicrogravityDetector {
public:
    static constexpr float    THRESHOLD_G = 0.1f;
    static constexpr uint32_t HOLD_MS     = 1000;
    static constexpr uint32_t MAX_GAP_MS  = 200;

    /// Einen Messwert in g einspeisen.
    void feed(float ax_g, float ay_g, float az_g, uint32_t now_ms) {
        if (ax_g == 0.0f && ay_g == 0.0f && az_g == 0.0f) {   // Sensor liefert nichts
            below_ = false;
            return;
        }
        if (have_last_ && static_cast<int32_t>(now_ms - last_ms_) > static_cast<int32_t>(MAX_GAP_MS)) {
            below_ = false;
        }
        have_last_ = true;
        last_ms_   = now_ms;

        const float mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
        if (mag >= THRESHOLD_G) {
            below_ = false;
            return;
        }
        if (!below_) {
            below_    = true;
            varied_   = false;
            since_ms_ = now_ms;
            first_[0] = ax_g; first_[1] = ay_g; first_[2] = az_g;
            return;
        }
        if (ax_g != first_[0] || ay_g != first_[1] || az_g != first_[2]) varied_ = true;
    }

    /// Schwerelos seit mindestens HOLD_MS und der letzte Wert ist frisch.
    bool active(uint32_t now_ms) const {
        if (!below_ || !varied_ || !have_last_) return false;
        if (static_cast<int32_t>(now_ms - last_ms_) > static_cast<int32_t>(MAX_GAP_MS)) return false;
        return static_cast<int32_t>(now_ms - since_ms_) >= static_cast<int32_t>(HOLD_MS);
    }

private:
    bool     below_     = false;
    bool     varied_    = false;
    bool     have_last_ = false;
    uint32_t since_ms_  = 0;
    uint32_t last_ms_   = 0;
    float    first_[3]  = {};
};
