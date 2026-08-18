#include "hal/motor_hal.hpp"

MotorHAL::MotorHAL(uint8_t pin_a, uint8_t pin_b, uint8_t motor_id)
    : pin_a_(pin_a), pin_b_(pin_b), motor_id_(motor_id) {
}

MotorHAL::~MotorHAL() {
    delete enc_;
}

bool MotorHAL::init() {
    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);

    setPWMFrequency(20000); // above audible range, within DRV8871 spec

    analogWrite(pin_a_, 0);
    analogWrite(pin_b_, 0);

    initialized_ = true;
    return true;
}

bool MotorHAL::initEncoder(uint8_t pin_enc_a, uint8_t pin_enc_b) {
    if (enc_) return true;                 // bereits initialisiert
    enc_ = new Encoder(pin_enc_a, pin_enc_b);
    enc_->write(0);                        // Nullpunkt setzen
    return enc_ != nullptr;
}

void MotorHAL::setSpeed(int16_t speed) {
    if (!initialized_) return;

    // Clamp speed to valid range
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    current_speed_ = speed;

    if (speed > 0) {
        // Forward: Channel A active, Channel B off
        analogWrite(pin_a_, speed);
        analogWrite(pin_b_, 0);
    } else if (speed < 0) {
        // Reverse: Channel B active, Channel A off
        analogWrite(pin_a_, 0);
        analogWrite(pin_b_, -speed);
    } else {
        // Stop: Both channels off
        analogWrite(pin_a_, 0);
        analogWrite(pin_b_, 0);
    }
}

void MotorHAL::stop() {
    setSpeed(0);
}

void MotorHAL::setPWMFrequency(uint32_t frequency) {
    analogWriteFrequency(pin_a_, frequency);
    analogWriteFrequency(pin_b_, frequency);
}

// ---------------------------------------------------------------------------
// Closed-Loop Positionsregelung
// ---------------------------------------------------------------------------

long MotorHAL::getPosition() {
    return enc_ ? enc_->read() : 0;
}

void MotorHAL::moveTo(long target) {
    if (!enc_) return;      // ohne Encoder keine Positionsregelung
    target_ = target;
    moving_ = true;
}

void MotorHAL::halfTurn() {
    if (!enc_) return;
    moveTo(getPosition() + HALF_TURN);
}

void MotorHAL::on() {
    moving_ = false;                 // eventuelle Fahrt abbrechen
    is_on_  = true;
    setSpeed(DEFAULT_ON_SPEED);
}

void MotorHAL::off() {
    moving_ = false;
    is_on_  = false;
    setSpeed(0);
}

void MotorHAL::update() {
    if (!moving_ || !enc_) return;   // nur laufende Fahrten regeln

    const long pos   = enc_->read();
    const long error = target_ - pos;

    if (labs(error) <= POS_TOL) {    // Ziel erreicht
        setSpeed(0);
        moving_ = false;
        return;
    }

    int pwm = static_cast<int>(Kp * labs(error));
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    setSpeed(static_cast<int16_t>(error > 0 ? pwm : -pwm));
}
