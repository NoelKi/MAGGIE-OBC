#pragma once

#include <cstdint>
#include <Arduino.h>
#include <IntervalTimer.h>
#include <Encoder.h>

/**
 * @file motor_hal.hpp
 * @brief Hardware Abstraction Layer for Motor Control (DRV8871)
 *
 * Reine Steuerung (Open Loop) einer DRV8871-H-Bruecke ueber die Kanaele A/B.
 * Der Quadratur-Encoder haengt optional daran, wird aber NUR als Sensor
 * gelesen - es gibt keine Positionsregelung. Ein einmal gestarteter Motor
 * dreht, bis off() kommt.
 *
 * Die frueheren Fahrten auf Encoder-Ziel (moveTo/halfTurn/update mit P-Regler,
 * Stall- und Zeitueberwachung) sind entfallen: Der Bodentest soll die
 * Telecommand-Strecke und den Motor selbst pruefen, nicht die Regelung. Der
 * Encoder ist damit Messmittel statt Regelgroesse.
 */

class MotorHAL {
public:
    /// Counts pro voller Umdrehung - stammt aus dem Prototyp
    /// hardwareTest/motor.cpp und ist am verbauten Getriebemotor NICHT
    /// nachgemessen. Wird nur noch fuer die Winkelanzeige am Boden gebraucht
    /// (Gegenstueck: MAGGIE_SERVER/app/services/downlink_frame_parser.py).
    static constexpr long COUNTS_PER_REV = 4600;

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
     * @brief Quadratur-Encoder anhängen (reine Positionsmessung)
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
     * PWM-Timer liegt (z.B. 40/41 auf der Teensy 4.1). Mit der aktuellen
     * Belegung (18/19, beide QuadTimer) bleibt der Modus aus.
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
    // Encoder (reine Messung, siehe initEncoder)
    // -----------------------------------------------------------------------

    /** @brief Aktuelle Encoder-Position in Quadratur-Counts (0 ohne Encoder). */
    long getPosition();

    /**
     * @brief Setzt den Encoder-Zaehler auf 0 und stoppt den Motor.
     *
     * Keine Regelung - nur ein Nullpunkt fuer die Anzeige am Boden. Damit
     * laesst sich COUNTS_PER_REV am Tisch nachmessen: nullen, eine
     * Wellenumdrehung drehen lassen, Counts ablesen.
     */
    void zeroPosition();

    /**
     * @brief Motor dauerhaft drehen lassen.
     * @param speed -255..+255, Vorzeichen = Drehrichtung.
     *              0 nimmt DEFAULT_ON_SPEED (vorwaerts).
     */
    void on(int16_t speed = 0);

    /** @brief Motor ausschalten. */
    void off();

    bool isOn() const { return is_on_; }          ///< Dauer-An/Aus-Zustand (on()/off())
    bool hasEncoder() const { return enc_ != nullptr; }

private:
    uint8_t pin_a_;
    uint8_t pin_b_;
    uint8_t motor_id_;
    int16_t current_speed_ = 0;
    bool initialized_ = false;

    Encoder* enc_ = nullptr;   ///< Quadratur-Encoder (nullptr = keine Messung)
    bool is_on_  = false;      ///< logischer An/Aus-Zustand (on()/off())

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

    /// Geschwindigkeit fuer on() ohne Argument. Muss ueber dem Losbrechmoment
    /// liegen, sonst brummt der Motor nur.
    static constexpr int16_t DEFAULT_ON_SPEED = 120;
};
