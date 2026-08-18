#pragma once

#include <cstdint>
#include <Arduino.h>
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
     */
    void setPWMFrequency(uint32_t frequency);

    // -----------------------------------------------------------------------
    // Closed-Loop Positionsregelung (benötigt Encoder, siehe initEncoder)
    // -----------------------------------------------------------------------

    /** @brief Aktuelle Encoder-Position in Quadratur-Counts (0 ohne Encoder). */
    long getPosition();

    /** @brief Startet eine geregelte Fahrt auf die absolute Zielposition. */
    void moveTo(long target);

    /** @brief Dreht eine halbe Umdrehung (+HALF_TURN) ab der aktuellen Position. */
    void halfTurn();

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

    // P-Regler-Parameter (aus hardwareTest/motor.cpp).
    static constexpr float   Kp        = 0.8f;   ///< Regler-Verstärkung
    static constexpr int     MIN_PWM   = 10;     ///< Losbrech-PWM
    static constexpr int     MAX_PWM   = 60;     ///< Obergrenze der Regel-PWM
    static constexpr long    POS_TOL   = 5;      ///< Zielfenster in Counts
    static constexpr int16_t DEFAULT_ON_SPEED = 120; ///< Geschwindigkeit für on()
};
