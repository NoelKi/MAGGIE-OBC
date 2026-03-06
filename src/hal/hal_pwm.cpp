#include "hal_pwm.h"
#include "hal_config.h"

namespace HAL {

static uint8_t currentSpeed = 0;
static MotorDirection currentDirection = MOTOR_STOP;
static bool motorEnabled = false;

bool pwmInit() {
    try {
        // Setze Pins als Output
        pinMode(HAL_PWM_SPEED, OUTPUT);
        pinMode(HAL_PWM_DIR1, OUTPUT);
        pinMode(HAL_PWM_DIR2, OUTPUT);
        pinMode(HAL_PWM_ENABLE, OUTPUT);
        
        // Starte mit Motor aus
        digitalWrite(HAL_PWM_ENABLE, LOW);
        digitalWrite(HAL_PWM_DIR1, LOW);
        digitalWrite(HAL_PWM_DIR2, LOW);
        analogWrite(HAL_PWM_SPEED, 0);
        
        Serial.println("[HAL_PWM] PWM Motor Control initialized");
        Serial.println("  Speed Pin: " + String(HAL_PWM_SPEED));
        Serial.println("  Direction Pins: " + String(HAL_PWM_DIR1) + ", " + String(HAL_PWM_DIR2));
        Serial.println("  Enable Pin: " + String(HAL_PWM_ENABLE));
        
        return true;
    } catch (...) {
        Serial.println("[HAL_PWM] ERROR: Failed to initialize PWM");
        return false;
    }
}

bool pwmSetSpeed(uint8_t speed) {
    if (speed > 100) speed = 100;  // Max 100%
    
    currentSpeed = speed;
    
    // Konvertiere 0-100% zu 0-255 (PWM)
    uint8_t pwmValue = (speed * 255) / 100;
    
    try {
        analogWrite(HAL_PWM_SPEED, pwmValue);
        return true;
    } catch (...) {
        return false;
    }
}

bool pwmSetDirection(MotorDirection direction) {
    currentDirection = direction;
    
    try {
        switch (direction) {
            case MOTOR_FORWARD:
                digitalWrite(HAL_PWM_DIR1, HIGH);
                digitalWrite(HAL_PWM_DIR2, LOW);
                break;
            
            case MOTOR_BACKWARD:
                digitalWrite(HAL_PWM_DIR1, LOW);
                digitalWrite(HAL_PWM_DIR2, HIGH);
                break;
            
            case MOTOR_BRAKE:
                // Beide Richtungen aktiv = Bremse
                digitalWrite(HAL_PWM_DIR1, HIGH);
                digitalWrite(HAL_PWM_DIR2, HIGH);
                break;
            
            case MOTOR_STOP:
            default:
                digitalWrite(HAL_PWM_DIR1, LOW);
                digitalWrite(HAL_PWM_DIR2, LOW);
                break;
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

bool pwmSetEnabled(bool enabled) {
    motorEnabled = enabled;
    
    try {
        digitalWrite(HAL_PWM_ENABLE, enabled ? HIGH : LOW);
        return true;
    } catch (...) {
        return false;
    }
}

bool pwmSetMotor(uint8_t speed, MotorDirection direction) {
    if (!pwmSetSpeed(speed)) return false;
    if (!pwmSetDirection(direction)) return false;
    return true;
}

bool pwmEmergencyStop() {
    try {
        pwmSetSpeed(0);           // Speed auf 0
        pwmSetDirection(MOTOR_BRAKE);  // Bremse
        pwmSetEnabled(false);      // Motor aus
        
        Serial.println("[HAL_PWM] EMERGENCY STOP activated!");
        return true;
    } catch (...) {
        return false;
    }
}

void pwmDebugStatus() {
    Serial.println("[HAL_PWM] Motor Status:");
    Serial.println("  Speed: " + String(currentSpeed) + "%");
    Serial.println("  Direction: " + String(currentDirection));
    Serial.println("  Enabled: " + String(motorEnabled ? "YES" : "NO"));
}

} // namespace HAL
