#pragma once
#include <Arduino.h>

/**
 * HAL PWM Interface - Abstrahiert PWM Motor Control
 * 
 * PWM-Pins (für HDRM Motor):
 * - Pin 9: Speed (0-255, entspricht 0-100%)
 * - Pin 7: Direction 1 (Forward)
 * - Pin 8: Direction 2 (Backward)
 * - Pin 6: Enable (Motor On/Off)
 */

namespace HAL {

// Motor Directions
enum MotorDirection {
    MOTOR_STOP = 0,
    MOTOR_FORWARD = 1,
    MOTOR_BACKWARD = 2,
    MOTOR_BRAKE = 3
};

/**
 * Initialisiert PWM für Motor Control
 * @return true wenn erfolgreich
 */
bool pwmInit();

/**
 * Setzt Motor-Geschwindigkeit
 * @param speed 0-100 (%)
 * @return true wenn erfolgreich
 */
bool pwmSetSpeed(uint8_t speed);

/**
 * Setzt Motor-Richtung
 * @param direction MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_STOP, MOTOR_BRAKE
 * @return true wenn erfolgreich
 */
bool pwmSetDirection(MotorDirection direction);

/**
 * Aktiviert/Deaktiviert Motor
 * @param enabled true = Motor an, false = Motor aus
 * @return true wenn erfolgreich
 */
bool pwmSetEnabled(bool enabled);

/**
 * Setzt Speed und Direction in einem Befehl
 * @param speed 0-100 (%)
 * @param direction MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_STOP, MOTOR_BRAKE
 * @return true wenn erfolgreich
 */
bool pwmSetMotor(uint8_t speed, MotorDirection direction);

/**
 * Emergency Stop - Motor sofort stoppen und bremsen
 * @return true wenn erfolgreich
 */
bool pwmEmergencyStop();

/**
 * Gibt aktuellen Motor-Status aus
 */
void pwmDebugStatus();

} // namespace HAL
