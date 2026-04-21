#include "hal_pwm.hpp"
#include "hal_config.hpp"

namespace HAL {

static uint8_t currentSpeed = 0;
static MotorDirection currentDirection = MOTOR_STOP;
static bool motorEnabled = false;

bool pwmInit() {
    pinMode(HAL_PWM_SPEED, OUTPUT);
    pinMode(HAL_PWM_DIR1, OUTPUT);
    pinMode(HAL_PWM_DIR2, OUTPUT);
    pinMode(HAL_PWM_ENABLE, OUTPUT);

    digitalWrite(HAL_PWM_ENABLE, LOW);
    digitalWrite(HAL_PWM_DIR1, LOW);
    digitalWrite(HAL_PWM_DIR2, LOW);
    analogWrite(HAL_PWM_SPEED, 0);

    Serial.println("[HAL_PWM] PWM Motor Control initialized");
    return true;
}

bool pwmSetSpeed(uint8_t speed) {
    if (speed > 100) speed = 100;
    currentSpeed = speed;
    uint8_t pwmValue = (speed * 255) / 100;
    analogWrite(HAL_PWM_SPEED, pwmValue);
    return true;
}

bool pwmSetDirection(MotorDirection direction) {
    currentDirection = direction;

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
}

bool pwmSetEnabled(bool enabled) {
    motorEnabled = enabled;
    digitalWrite(HAL_PWM_ENABLE, enabled ? HIGH : LOW);
    return true;
}

bool pwmSetMotor(uint8_t speed, MotorDirection direction) {
    if (!pwmSetSpeed(speed)) return false;
    if (!pwmSetDirection(direction)) return false;
    return true;
}

bool pwmEmergencyStop() {
    pwmSetSpeed(0);
    pwmSetDirection(MOTOR_BRAKE);
    pwmSetEnabled(false);
    Serial.println("[HAL_PWM] EMERGENCY STOP activated!");
    return true;
}

void pwmDebugStatus() {
    char buf[80];
    snprintf(buf, sizeof(buf), "[HAL_PWM] Speed: %d%%  Dir: %d  Enabled: %s",
             currentSpeed, (int)currentDirection, motorEnabled ? "YES" : "NO");
    Serial.println(buf);
}

} // namespace HAL
