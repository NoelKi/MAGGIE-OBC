#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief PWM-Motor Steuerungsdriver für HDRM
 * Steuert Motor-Drehzahl und Richtung über PWM
 * 
 * Verwendung:
 * - Speed Control (0-255 oder 0-100%)
 * - Richtungssteuerung (Vorwärts/Rückwärts/Stop)
 * - PWM-Frequenz konfigurierbar
 */

enum class MotorDirection {
    FORWARD,     // Vorwärts
    BACKWARD,    // Rückwärts
    STOP,        // Stopp
    BRAKE,       // Bremse
};

class PWMMotorDriver {
public:
    /**
     * @brief Konstruktor
     * @param pwm_pin PWM-Pin für Geschwindigkeit
     * @param dir_pin1 Richtungs-Pin 1 (vorwärts)
     * @param dir_pin2 Richtungs-Pin 2 (rückwärts)
     * @param enable_pin Enable-Pin (optional)
     */
    PWMMotorDriver(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2, uint8_t enable_pin = 255);
    ~PWMMotorDriver();
    
    /**
     * @brief Initialisiert Motor-Pins und PWM
     * @param pwm_frequency PWM-Frequenz in Hz (Default: 1 kHz)
     * @return true wenn erfolgreich
     */
    bool init(uint32_t pwm_frequency = 1000);
    
    /**
     * @brief Setzt Motor-Geschwindigkeit (0-100%)
     * @param speed Geschwindigkeit 0-100%
     * @param direction Richtung
     */
    void setSpeed(uint8_t speed, MotorDirection direction = MotorDirection::FORWARD);
    
    /**
     * @brief Setzt Motor-Geschwindigkeit (0-255 PWM)
     * @param pwm_value PWM-Wert 0-255
     * @param direction Richtung
     */
    void setPWM(uint8_t pwm_value, MotorDirection direction = MotorDirection::FORWARD);
    
    /**
     * @brief Stoppt Motor sofort
     */
    void stop();
    
    /**
     * @brief Bremst Motor mit vollem PWM (dynamische Bremse)
     */
    void brake();
    
    /**
     * @brief Gibt aktuelle Geschwindigkeit zurück (0-255)
     */
    uint8_t getSpeed() const;
    
    /**
     * @brief Gibt aktuelle Richtung zurück
     */
    MotorDirection getDirection() const;
    
    /**
     * @brief Prüft ob Motor läuft
     */
    bool isRunning() const;

private:
    uint8_t pwm_pin;
    uint8_t dir_pin1;
    uint8_t dir_pin2;
    uint8_t enable_pin;
    
    uint8_t current_speed = 0;
    MotorDirection current_direction = MotorDirection::STOP;
    bool motor_enabled = false;
    
    void updateMotor();
};
