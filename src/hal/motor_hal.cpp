#include "hal/motor_hal.hpp"

// ---------------------------------------------------------------------------
// Software-PWM: Statics (siehe motor_hal.hpp)
// ---------------------------------------------------------------------------
IntervalTimer    MotorHAL::soft_timer_;
volatile uint8_t MotorHAL::soft_duty_a_ = 0;
volatile uint8_t MotorHAL::soft_duty_b_ = 0;
volatile uint8_t MotorHAL::soft_tick_   = 0;
uint8_t          MotorHAL::soft_pin_a_  = 255;
uint8_t          MotorHAL::soft_pin_b_  = 255;
bool             MotorHAL::soft_active_ = false;

bool MotorHAL::pinHasHardwarePwm(uint8_t pin) {
    // Teensy-4.1-Pins ohne FlexPWM-/QuadTimer-Kanal. Ermittelt aus
    // cores/teensy4/pwm.c, Tabelle pwm_pin_info[] (Eintraege mit type == 0);
    // dort laeuft analogWrite() in ein stilles `return`.
    static constexpr uint8_t NO_PWM[] = {
        16, 17, 20, 21, 26, 27, 30, 31, 32,
        34, 35, 38, 39, 40, 41, 48, 49, 50, 52, 53,
    };
    for (uint8_t p : NO_PWM) {
        if (p == pin) return false;
    }
    return pin < 55;   // CORE_NUM_DIGITAL der Teensy 4.1
}

void MotorHAL::softPwmIsr() {
    // soft_tick_ ist uint8_t und laeuft bei 256 von selbst ueber -> eine
    // volle PWM-Periode entspricht SOFT_PWM_STEPS Ticks.
    const uint8_t t = soft_tick_++;
    digitalWrite(soft_pin_a_, t < soft_duty_a_ ? HIGH : LOW);
    digitalWrite(soft_pin_b_, t < soft_duty_b_ ? HIGH : LOW);
}

void MotorHAL::softPwmBegin(uint32_t frequency) {
    if (frequency < SOFT_PWM_HZ_MIN) frequency = SOFT_PWM_HZ_MIN;
    if (frequency > SOFT_PWM_HZ_MAX) frequency = SOFT_PWM_HZ_MAX;

    soft_pin_a_ = pin_a_;
    soft_pin_b_ = pin_b_;

    const float tick_us =
        1000000.0f / static_cast<float>(frequency * SOFT_PWM_STEPS);

    if (soft_active_) soft_timer_.end();
    soft_timer_.begin(softPwmIsr, tick_us);
    soft_timer_.priority(SOFT_PWM_PRIORITY);
    soft_active_ = true;
}

void MotorHAL::writeChannels(uint8_t duty_a, uint8_t duty_b) {
    if (soft_pwm_) {
        soft_duty_a_ = duty_a;
        soft_duty_b_ = duty_b;
    } else {
        analogWrite(pin_a_, duty_a);
        analogWrite(pin_b_, duty_b);
    }
}

MotorHAL::MotorHAL(uint8_t pin_a, uint8_t pin_b, uint8_t motor_id)
    : pin_a_(pin_a), pin_b_(pin_b), motor_id_(motor_id) {
}

MotorHAL::~MotorHAL() {
    if (soft_pwm_ && soft_active_) {
        soft_timer_.end();
        soft_active_ = false;
    }
    delete enc_;
}

bool MotorHAL::init() {
    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);

    // Liegt einer der Kanaele auf einem Pin ohne PWM-Timer (z.B. 40/41 auf der
    // 4.1), waere analogWrite() wirkungslos und der Motor bliebe ohne jede
    // Fehlermeldung stehen. In dem Fall uebernimmt der IntervalTimer.
    soft_pwm_ = !pinHasHardwarePwm(pin_a_) || !pinHasHardwarePwm(pin_b_);

    if (soft_pwm_) {
        digitalWrite(pin_a_, LOW);
        digitalWrite(pin_b_, LOW);
        softPwmBegin(SOFT_PWM_HZ_DEF);
    } else {
        setPWMFrequency(20000); // above audible range, within DRV8871 spec
        writeChannels(0, 0);
    }

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
        writeChannels(static_cast<uint8_t>(speed), 0);
    } else if (speed < 0) {
        // Reverse: Channel B active, Channel A off
        writeChannels(0, static_cast<uint8_t>(-speed));
    } else {
        // Stop: Both channels off
        writeChannels(0, 0);
    }
}

void MotorHAL::stop() {
    setSpeed(0);
}

void MotorHAL::setPWMFrequency(uint32_t frequency) {
    if (soft_pwm_) {
        softPwmBegin(frequency);   // begrenzt intern auf SOFT_PWM_HZ_MIN..MAX
        return;
    }
    analogWriteFrequency(pin_a_, frequency);
    analogWriteFrequency(pin_b_, frequency);
}

// ---------------------------------------------------------------------------
// Encoder (reine Messung) und Dauerbetrieb
// ---------------------------------------------------------------------------

long MotorHAL::getPosition() {
    return enc_ ? enc_->read() : 0;
}

void MotorHAL::zeroPosition() {
    setSpeed(0);
    is_on_ = false;
    if (enc_) enc_->write(0);
}

void MotorHAL::on(int16_t speed) {
    if (speed == 0) speed = DEFAULT_ON_SPEED;
    if (speed >  255) speed =  255;
    if (speed < -255) speed = -255;
    is_on_ = true;
    setSpeed(speed);
}

void MotorHAL::off() {
    is_on_ = false;
    setSpeed(0);
}
