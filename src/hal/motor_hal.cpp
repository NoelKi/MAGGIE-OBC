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

    // Untergrenze: Darunter fliesst zwar Strom, der Motor laeuft aber nicht an
    // (Haftreibung + Getriebe). Anheben statt fahren zu lassen - ein
    // stehender Motor unter Strom heizt nur die Bruecke. 0 bleibt Stopp.
    if (speed > 0 && speed <  MIN_DRIVE_SPEED) speed =  MIN_DRIVE_SPEED;
    if (speed < 0 && speed > -MIN_DRIVE_SPEED) speed = -MIN_DRIVE_SPEED;

    current_speed_ = speed;
    braking_ = false;          // ein neuer Fahrbefehl beendet die Bremsphase

    if (speed > 0) {
        // Forward: Channel A active, Channel B off (DRV8871 Table 1: 1/0)
        writeChannels(static_cast<uint8_t>(speed), 0);
    } else if (speed < 0) {
        // Reverse: Channel B active, Channel A off (Table 1: 0/1)
        writeChannels(0, static_cast<uint8_t>(-speed));
    } else {
        // Coast: beide Kanaele LOW -> H-Bruecke hochohmig (Table 1: 0/0)
        writeChannels(0, 0);
    }
}

void MotorHAL::stop() {
    setSpeed(0);
}

void MotorHAL::brake() {
    if (!initialized_) return;

    // DRV8871 Table 1: IN1=1, IN2=1 -> "Brake; low-side slow decay". Die
    // Motorklemmen werden kurzgeschlossen, der Anker bremst gegen sich selbst.
    // Mit Coast (0/0) laeuft der Motor stattdessen frei aus - das war die
    // Hauptquelle des Ueberschwingens am Ende einer Drehung.
    writeChannels(255, 255);
    current_speed_  = 0;
    braking_        = true;
    brake_start_ms_ = millis();
}

void MotorHAL::updateBrake() {
    // Nur ein kurzer Bremsimpuls: Dauerhaft gebremst bliebe die H-Bruecke
    // aktiv (kein Sleep) und die Mechanik liesse sich nicht von Hand bewegen.
    // Nach BRAKE_MS zurueck auf Coast.
    if (!braking_) return;
    if (millis() - brake_start_ms_ < BRAKE_MS) return;

    braking_ = false;
    writeChannels(0, 0);
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
    off();
    if (enc_) enc_->write(0);
}

void MotorHAL::on(int16_t speed) {
    if (speed == 0) speed = DEFAULT_ON_SPEED;
    if (speed >  255) speed =  255;
    if (speed < -255) speed = -255;
    turning_ = false;          // Handbetrieb schlaegt eine laufende Drehung
    is_on_   = true;
    setSpeed(speed);
}

void MotorHAL::off() {
    turning_ = false;
    is_on_   = false;
    brake();
}

void MotorHAL::turnBy(int16_t degrees) {
    // Ohne Encoder gaebe es kein Abschaltkriterium - der Motor liefe bis zum
    // MOTOR_OFF bzw. bis der Laufzeit-Watchdog in System zugreift. Lieber gar
    // nicht erst anfahren.
    if (!enc_ || degrees == 0) return;

    const long delta = static_cast<long>(degrees) * COUNTS_PER_REV / 360;
    if (delta == 0) return;    // Winkel zu klein fuer einen ganzen Count

    startMove(enc_->read() + delta, degrees, "relativ");
}

void MotorHAL::goTo(int16_t degrees) {
    if (!enc_) return;

    // ABSOLUT zur Encoder-Null. Der entscheidende Unterschied zu turnBy():
    // Das Ziel haengt nicht davon ab, wo die letzte Fahrt geendet hat. Ein
    // Ueberschwinger wird bei der naechsten Fahrt automatisch mit
    // ausgeglichen, statt sich Zyklus fuer Zyklus aufzuaddieren - genau das
    // laesst die Nullage bei relativen Fahrten wandern.
    const long target = static_cast<long>(degrees) * COUNTS_PER_REV / 360;
    const long pos    = enc_->read();

    // Totband: Ohne das wuerde ein erneuter Befehl auf dieselbe Position den
    // Motor um den Zielpunkt herum pendeln lassen - die Fahrt stoppt ja erst
    // NACH dem Ueberschreiten des Ziels, steht also immer ein Stueck dahinter.
    if (labs(target - pos) <= POS_DEADBAND) {
        Serial.printf("INFO  [MotorHAL]: Position %ld schon im Zielfenster "
                      "(%ld +/- %ld Counts) - keine Fahrt.\n",
                      pos, target, POS_DEADBAND);
        return;
    }

    startMove(target, degrees, "absolut");
}

void MotorHAL::startMove(long target, int16_t degrees, const char* kind) {
    const long start = enc_->read();

    on(target > start ? TURN_SPEED : -TURN_SPEED);   // setzt turning_ zurueck
    turn_start_    = start;
    turn_target_   = target;
    turn_ref_pos_  = start;
    turn_ref_ms_   = millis();
    turn_failed_   = false;
    turning_       = true;

    Serial.printf("INFO  [MotorHAL]: Fahrt %s %d Grad - Start %ld, Ziel %ld "
                  "(%ld Counts).\n",
                  kind, degrees, turn_start_, turn_target_, target - start);
}

void MotorHAL::update() {
    updateTurn();
    updateBrake();
}

void MotorHAL::updateTurn() {
    if (!turning_ || !enc_) return;

    // Vergleich in Fahrtrichtung, nicht ueber den Betrag: Ein simples
    // labs(pos - target) <= Toleranz wuerde bei zu grosser Schrittweite
    // zwischen zwei Loops uebersprungen und die Drehung liefe endlos weiter.
    const long pos     = enc_->read();
    const bool forward = current_speed_ > 0;
    const bool reached = forward ? pos >= turn_target_ : pos <= turn_target_;

    if (!reached) {
        // Fortschritt IN Fahrtrichtung. Faengt beide Ausfaelle ab, bei denen das
        // Ziel nie erreichbar ist und der Motor sonst bis zum Laufzeit-Watchdog
        // durchliefe: Encoder zaehlt gar nicht (Kanal ab, falscher Pin) oder er
        // zaehlt verkehrt herum (A/B vertauscht) - dann laeuft die Position vom
        // Ziel weg.
        const long progress = forward ? pos - turn_ref_pos_ : turn_ref_pos_ - pos;

        if (progress >= TURN_MIN_COUNTS) {
            turn_ref_pos_ = pos;             // Fortschritt -> Fenster neu aufziehen
            turn_ref_ms_  = millis();
        } else if (millis() - turn_ref_ms_ >= TURN_STALL_MS) {
            off();
            turn_failed_ = true;
            Serial.printf(
                "WARN  [MotorHAL]: Drehung abgebrochen - in %lu ms nur %ld Counts "
                "in Fahrtrichtung. Position %ld (Start %ld, Ziel %ld, PWM %d).\n",
                static_cast<unsigned long>(TURN_STALL_MS), progress,
                pos, turn_start_, turn_target_, current_speed_);
            Serial.println(
                "WARN  [MotorHAL]: Position unveraendert -> Encoder zaehlt nicht "
                "(Verdrahtung/Pins pruefen). Position laeuft weg -> Encoder-Kanaele "
                "A/B vertauscht. Mechanik fest -> Motor blockiert.");
        }
        return;
    }

    off();
    // Gedreht = tatsaechlich zurueckgelegte Counts. Weicht der Wert deutlich vom
    // Sollbetrag ab, stimmt COUNTS_PER_REV nicht - genau der Vergleich, den man
    // zum Kalibrieren braucht.
    Serial.printf("INFO  [MotorHAL]: Drehung beendet - Position %ld (Ziel %ld, "
                  "gedreht %ld Counts = %.1f Grad bei COUNTS_PER_REV=%ld).\n",
                  pos, turn_target_, pos - turn_start_,
                  static_cast<double>(pos - turn_start_) * 360.0 / COUNTS_PER_REV,
                  COUNTS_PER_REV);
}
