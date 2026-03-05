#include "drivers/pwm_motor_driver.h"

PWMMotorDriver::PWMMotorDriver(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2, uint8_t enable_pin)
    : pwm_pin(pwm_pin), dir_pin1(dir_pin1), dir_pin2(dir_pin2), enable_pin(enable_pin) {
    // Konstruktor
}

PWMMotorDriver::~PWMMotorDriver() {
    // Destruktor
}

bool PWMMotorDriver::init(uint32_t pwm_frequency) {
    // Konfiguriere Pins als Ausgänge
    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin1, OUTPUT);
    pinMode(dir_pin2, OUTPUT);
    
    if (enable_pin != 255) {
        pinMode(enable_pin, OUTPUT);
        digitalWrite(enable_pin, HIGH);  // Enable aktivieren
    }
    
    // Initialisiere Motor auf Stop
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, LOW);
    analogWrite(pwm_pin, 0);
    
    motor_enabled = true;
    
    Serial.printf("INFO: PWM Motor initialized on pin %d (Dir: %d/%d, Freq: %lu Hz)\n",
                 pwm_pin, dir_pin1, dir_pin2, pwm_frequency);
    
    // TODO: Setze PWM-Frequenz auf Teensy 4.1
    // analogWriteFrequency(pwm_pin, pwm_frequency);
    
    return true;
}

void PWMMotorDriver::setSpeed(uint8_t speed, MotorDirection direction) {
    if (speed > 100) speed = 100;
    
    // Konvertiere Prozent zu PWM (0-255)
    uint8_t pwm_value = (speed * 255) / 100;
    setPWM(pwm_value, direction);
}

void PWMMotorDriver::setPWM(uint8_t pwm_value, MotorDirection direction) {
    if (!motor_enabled) return;
    
    current_speed = pwm_value;
    current_direction = direction;
    
    updateMotor();
}

void PWMMotorDriver::stop() {
    current_speed = 0;
    current_direction = MotorDirection::STOP;
    updateMotor();
}

void PWMMotorDriver::brake() {
    current_speed = 255;
    current_direction = MotorDirection::BRAKE;
    updateMotor();
}

uint8_t PWMMotorDriver::getSpeed() const {
    return current_speed;
}

MotorDirection PWMMotorDriver::getDirection() const {
    return current_direction;
}

bool PWMMotorDriver::isRunning() const {
    return current_speed > 0 && current_direction != MotorDirection::STOP;
}

void PWMMotorDriver::updateMotor() {
    switch (current_direction) {
        case MotorDirection::FORWARD:
            digitalWrite(dir_pin1, HIGH);
            digitalWrite(dir_pin2, LOW);
            analogWrite(pwm_pin, current_speed);
            Serial.printf("MOTOR: Forward at %d%% PWM\n", (current_speed * 100) / 255);
            break;
            
        case MotorDirection::BACKWARD:
            digitalWrite(dir_pin1, LOW);
            digitalWrite(dir_pin2, HIGH);
            analogWrite(pwm_pin, current_speed);
            Serial.printf("MOTOR: Backward at %d%% PWM\n", (current_speed * 100) / 255);
            break;
            
        case MotorDirection::STOP:
            digitalWrite(dir_pin1, LOW);
            digitalWrite(dir_pin2, LOW);
            analogWrite(pwm_pin, 0);
            Serial.println("MOTOR: Stopped");
            break;
            
        case MotorDirection::BRAKE:
            // Dynamische Bremse: beide Richtungs-Pins HIGH
            digitalWrite(dir_pin1, HIGH);
            digitalWrite(dir_pin2, HIGH);
            analogWrite(pwm_pin, current_speed);
            Serial.println("MOTOR: Braking");
            break;
    }
}
