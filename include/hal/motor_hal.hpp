#pragma once

#include <cstdint>
#include <Arduino.h>
#include <IntervalTimer.h>
#include <Encoder.h>

/**
 * @file motor_hal.hpp
 * @brief Hardware Abstraction Layer for Motor Control (DRV8871)
 *
 * Provides PWM control for a DRV8871 H-bridge motor (channels A/B) and,
 * optionally, closed-loop position control via a quadrature encoder.
 *
 * Die Positionsregelung (moveTo/halfTurn/update) stammt aus dem getesteten
 * Prototyp hardwareTest/motor.cpp, wurde aber von einer blockierenden
 * while-Schleife auf einen nicht-blockierenden Schritt (update()) umgestellt,
 * damit sie im normalen System::run()-Loop laufen kann.
 */

class MotorHAL {
public:
    // Positionsregelung: an den verbauten Encoder/Getriebe angepasst
    // (kalibriert in hardwareTest/motor.cpp).
    static constexpr long COUNTS_PER_REV = 4600;                ///< Counts pro voller Umdrehung
    static constexpr long HALF_TURN      = COUNTS_PER_REV / 2;  ///< 180° ≈ 2300 Counts

    // HDRM (Hold-Down & Release Mechanism): der Mechanismus wird durch eine
    // halbe Umdrehung geöffnet. Referenz ist die Encoder-Nullposition, die
    // beim Start bzw. per zeroPosition() gesetzt wird -> die Fahrten sind
    // ABSOLUT, mehrfaches Öffnen dreht den Motor also nicht weiter.
    static constexpr long HDRM_CLOSED_COUNTS = 0;               ///< Nullposition = verriegelt
    static constexpr long HDRM_OPEN_COUNTS   = HALF_TURN;       ///< 180° = freigegeben
    static constexpr long HDRM_TOL_COUNTS    = 60;              ///< Fenster für die Zustandsmeldung

    /**
     * @brief Constructor for Motor HAL
     * @param pin_a Channel A (PWM)
     * @param pin_b Channel B (PWM)
     * @param motor_id Motor identifier (1-3)
     */
    MotorHAL(uint8_t pin_a, uint8_t pin_b, uint8_t motor_id = 0);
    ~MotorHAL();

    /**
     * @brief Initialize motor pins (open-loop)
     * @return true if successful
     */
    bool init();

    /**
     * @brief Quadratur-Encoder anhängen (aktiviert die Closed-Loop-Regelung)
     * @param pin_enc_a Encoder Channel A
     * @param pin_enc_b Encoder Channel B
     * @return true if successful
     */
    bool initEncoder(uint8_t pin_enc_a, uint8_t pin_enc_b);

    /**
     * @brief Set motor speed and direction
     * @param speed -255 (full reverse) to +255 (full forward), 0 = stop
     */
    void setSpeed(int16_t speed);

    /**
     * @brief Stop motor immediately
     */
    void stop();

    /**
     * @brief Get current motor speed
     * @return Current speed value (-255..255)
     */
    int16_t getSpeed() const { return current_speed_; }

    /**
     * @brief Set PWM frequency
     * @param frequency Frequency in Hz
     *
     * Im Software-PWM-Modus (siehe usesSoftPwm) wird der Wert auf
     * SOFT_PWM_HZ_MIN..SOFT_PWM_HZ_MAX begrenzt - 20 kHz sind per
     * IntervalTimer nicht erreichbar.
     */
    void setPWMFrequency(uint32_t frequency);

    /**
     * @brief true, wenn dieser Motor per Software-PWM getaktet wird.
     *
     * Greift automatisch, sobald einer der beiden Kanaele auf einem Pin ohne
     * PWM-Timer liegt (z.B. 40/41 auf der Teensy 4.1).
     */
    bool usesSoftPwm() const { return soft_pwm_; }

    /**
     * @brief Hat dieser Teensy-4.1-Pin einen FlexPWM-/QuadTimer-Kanal?
     *
     * Auf Pins ohne Eintrag in pwm_pin_info[] kehrt analogWrite() wirkungslos
     * zurueck (cores/teensy4/pwm.c: `else { return; }`) - ohne Fehlermeldung.
     */
    static bool pinHasHardwarePwm(uint8_t pin);

    // -----------------------------------------------------------------------
    // Closed-Loop Positionsregelung (benötigt Encoder, siehe initEncoder)
    // -----------------------------------------------------------------------

    /** @brief Aktuelle Encoder-Position in Quadratur-Counts (0 ohne Encoder). */
    long getPosition();

    /** @brief Startet eine geregelte Fahrt auf die absolute Zielposition. */
    void moveTo(long target);

    /** @brief Dreht eine halbe Umdrehung (+HALF_TURN) ab der aktuellen Position. */
    void halfTurn();

    /** @brief Fährt den HDRM auf die absolute Offen-Position (+180°). */
    void hdrmOpen() { moveTo(HDRM_OPEN_COUNTS); }

    /** @brief Fährt den HDRM auf die absolute Geschlossen-Position (Nullpunkt). */
    void hdrmClose() { moveTo(HDRM_CLOSED_COUNTS); }

    /**
     * @brief Setzt die aktuelle Position als Nullpunkt ("HDRM geschlossen").
     * Bricht eine laufende Fahrt ab und stoppt den Motor.
     */
    void zeroPosition();

    /** @brief true, wenn die Position im Fenster um die Offen-Position liegt. */
    bool isHdrmOpen();

    /** @brief true, wenn die Position im Fenster um den Nullpunkt liegt. */
    bool isHdrmClosed();

    /** @brief Motor dauerhaft mit Default-Geschwindigkeit einschalten. */
    void on();

    /** @brief Motor ausschalten (stoppt und bricht eine laufende Fahrt ab). */
    void off();

    /**
     * @brief Regelungsschritt - muss zyklisch (jeden Loop) aufgerufen werden.
     * Führt bei einer laufenden moveTo()-Fahrt einen P-Regler-Schritt aus und
     * stoppt am Ziel. Ohne laufende Fahrt tut update() nichts.
     */
    void update();

    bool isOn() const { return is_on_; }          ///< Dauer-An/Aus-Zustand (on()/off())
    bool isMoving() const { return moving_; }      ///< Closed-Loop-Fahrt aktiv
    bool hasEncoder() const { return enc_ != nullptr; }

    /**
     * @brief Wurde die letzte Fahrt abgebrochen, statt das Ziel zu erreichen?
     * Wird bei jedem neuen moveTo() zurueckgesetzt. Siehe MOVE_TIMEOUT_MS.
     */
    bool moveFailed() const { return move_failed_; }

private:
    uint8_t pin_a_;
    uint8_t pin_b_;
    uint8_t motor_id_;
    int16_t current_speed_ = 0;
    bool initialized_ = false;

    Encoder* enc_ = nullptr;   ///< Quadratur-Encoder (nullptr = keine Regelung)
    bool is_on_  = false;      ///< logischer An/Aus-Zustand (on()/off())
    bool moving_ = false;      ///< eine geregelte Fahrt läuft
    long target_ = 0;          ///< Zielposition der laufenden Fahrt

    // Ueberwachung der laufenden Fahrt, siehe MOVE_TIMEOUT_MS/STALL_WINDOW_MS.
    uint32_t move_start_ms_ = 0;   ///< Start der laufenden Fahrt
    uint32_t stall_ref_ms_  = 0;   ///< Beginn des aktuellen Stillstandsfensters
    long     stall_ref_pos_ = 0;   ///< Position zu Beginn dieses Fensters
    bool     move_failed_   = false; ///< letzte Fahrt wurde abgebrochen

    /// Fahrt stoppen und als gescheitert markieren (mit Log-Ausgabe).
    void abortMove(const char* reason);

    // -----------------------------------------------------------------------
    // Software-PWM fuer Pins ohne Hardware-Timer
    // -----------------------------------------------------------------------
    // Ein IntervalTimer tickt SOFT_PWM_STEPS mal pro Periode und setzt die
    // beiden Kanaele per digitalWrite. Die Statics gelten fuer EINEN Motor -
    // aktuell wird auch nur Motor 1 instanziiert (system.cpp).
    static constexpr uint32_t SOFT_PWM_STEPS  = 256;   ///< Aufloesung = analogWrite-Bereich
    static constexpr uint32_t SOFT_PWM_HZ_DEF = 1000;  ///< Default-Traegerfrequenz
    static constexpr uint32_t SOFT_PWM_HZ_MIN = 100;
    static constexpr uint32_t SOFT_PWM_HZ_MAX = 2000;  ///< darueber wird die ISR-Last zu hoch
    /// Interrupt-Prioritaet: hoeherer Wert = niedrigere Prioritaet. Muss unter
    /// der des Encoders (Default 128) liegen, sonst gehen Quadratur-Flanken
    /// verloren und die Positionsregelung driftet.
    static constexpr uint8_t  SOFT_PWM_PRIORITY = 192;

    bool soft_pwm_ = false;    ///< dieser Motor laeuft auf Software-PWM

    static IntervalTimer   soft_timer_;
    static volatile uint8_t soft_duty_a_;
    static volatile uint8_t soft_duty_b_;
    static volatile uint8_t soft_tick_;
    static uint8_t soft_pin_a_;
    static uint8_t soft_pin_b_;
    static bool    soft_active_;

    static void softPwmIsr();
    void softPwmBegin(uint32_t frequency);

    /** @brief Schreibt beide Kanaele - je nach Modus per analogWrite oder Soft-PWM. */
    void writeChannels(uint8_t duty_a, uint8_t duty_b);

    // P-Regler-Parameter (aus hardwareTest/motor.cpp).
    static constexpr float   Kp        = 0.8f;   ///< Regler-Verstärkung
    static constexpr int     MIN_PWM   = 10;     ///< Losbrech-PWM
    static constexpr int     MAX_PWM   = 60;     ///< Obergrenze der Regel-PWM
    static constexpr long    POS_TOL   = 5;      ///< Zielfenster in Counts
    static constexpr int16_t DEFAULT_ON_SPEED = 120; ///< Geschwindigkeit für on()

    // -----------------------------------------------------------------------
    // Abbruchkriterien für geregelte Fahrten (moveTo/halfTurn/hdrmOpen/-Close)
    // -----------------------------------------------------------------------
    // Ohne diese Grenzen faehrt der Motor unbegrenzt weiter, sobald das Ziel
    // nicht erreichbar ist: update() sieht dann nie labs(error) <= POS_TOL.
    // Passiert bei totem oder unverdrahtetem Encoder (Position steht), bei
    // blockierter Mechanik und bei verpolten Encoder-Kanaelen (Position laeuft
    // vom Ziel weg). Fuer den HDRM heisst das: Motor im Anschlag, bis jemand
    // MOTOR_OFF sendet.
    static constexpr uint32_t MOVE_TIMEOUT_MS  = 8000;  ///< harte Obergrenze je Fahrt
    static constexpr uint32_t STALL_WINDOW_MS  = 1000;  ///< Fenster ohne Fortschritt
    static constexpr long     STALL_MIN_COUNTS = 3;     ///< Fortschritt, der als Bewegung zaehlt
};
