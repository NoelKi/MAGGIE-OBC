#include "hal/motor_hal.hpp"

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

void MotorHAL::writeChannels(uint8_t duty_a, uint8_t duty_b) {
    analogWrite(pin_a_, duty_a);
    analogWrite(pin_b_, duty_b);
}

MotorHAL::MotorHAL(uint8_t pin_a, uint8_t pin_b, uint8_t motor_id, bool soft_approach)
    : pin_a_(pin_a), pin_b_(pin_b), motor_id_(motor_id), soft_approach_(soft_approach) {
}

MotorHAL::~MotorHAL() {
    delete enc_;
}

bool MotorHAL::init() {
    // Ohne PWM-Timer waere analogWrite() wirkungslos und der Motor bliebe ohne
    // jede Fehlermeldung stehen - dann lieber als "nicht bereit" melden.
    if (!pinHasHardwarePwm(pin_a_) || !pinHasHardwarePwm(pin_b_)) return false;

    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);
    analogWriteFrequency(pin_a_, 20000);   // ueber dem Hoerbereich, im DRV8871-Spec
    analogWriteFrequency(pin_b_, 20000);
    writeChannels(0, 0);

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

    // KEINE Untergrenze mehr (Stand 2026-08-26): Bodentest, um die reale
    // Anlaufschwelle des Getriebemotors zu vermessen ("ab welchem PWM reagiert
    // er noch"). MIN_DRIVE_SPEED bleibt als TURN_SPEED/Anfahr-Zielwert fuer
    // turnBy()/goTo() bestehen - die rufen setSpeed() nie mit einem kleineren
    // Wert auf, sind also von dieser Aenderung nicht betroffen. Nur der
    // manuelle Dauerlauf (on()/MOTOR_ON) kann jetzt Werte unter
    // MIN_DRIVE_SPEED tatsaechlich ausgeben. Ein stehender Motor unter Strom
    // heizt weiterhin nur die Bruecke - Expositionszeit bleibt durch
    // MOTOR_ON_TIMEOUT_MS (System) auf max. 30 s begrenzt.
    current_speed_ = speed;
    braking_ = false;          // ein neuer Fahrbefehl beendet die Bremsphase
    locked_  = false;          // analogWrite holt die Pins vom GPIO zurueck auf PWM

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
    locked_         = false;
}

void MotorHAL::lock() {
    if (!initialized_) return;

    turning_       = false;
    is_on_         = false;
    braking_       = false;   // sonst schaltet updateBrake() nach BRAKE_MS auf Coast
    current_speed_ = 0;

    // Per digitalWrite statt analogWrite(255): Bei 8 Bit bleibt mit 255 ein
    // kurzer LOW-Puls je PWM-Periode. pinMode schaltet den Pin vom PWM-Timer
    // zurueck auf GPIO; der naechste analogWrite() holt ihn wieder.
    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);
    digitalWrite(pin_a_, HIGH);
    digitalWrite(pin_b_, HIGH);
    locked_ = true;
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

    // soft_approach_ cruist schneller als TURN_SPEED - die Reserve nach oben
    // ist der einzige Spielraum, weil TURN_SPEED == MIN_DRIVE_SPEED sonst
    // keine Rampe zulaesst (siehe SOFT_APPROACH_* in motor_hal.hpp).
    const int16_t cruise = soft_approach_ ? SOFT_APPROACH_CRUISE_SPEED : TURN_SPEED;
    on(target > start ? cruise : -cruise);           // setzt turning_ zurueck
    turn_start_    = start;
    turn_target_   = target;
    turn_ref_pos_  = start;
    turn_ref_ms_   = millis();
    turn_failed_   = false;
    turning_       = true;

    turn_dir_        = target > start ? 1 : -1;
    turn_cruise_     = cruise;
    breakaway_tries_ = 0;
    // Startet die Fahrt schon in der Anfahrzone, bedeutet Stillstand dort
    // "Anschlag erreicht" (siehe updateTurn) - dann kein Anlauf.
    turn_moving_ = soft_approach_ && labs(target - start) <= SOFT_APPROACH_WINDOW_COUNTS;
    if (!turn_moving_) startRunup(start);

    Serial.printf("INFO  [MotorHAL]: Fahrt %s %d Grad - Start %ld, Ziel %ld "
                  "(%ld Counts).\n",
                  kind, degrees, turn_start_, turn_target_, target - start);
}

void MotorHAL::update() {
    updateTurn();
    updateBrake();
}

void MotorHAL::startRunup(long pos) {
    breakaway_tries_++;
    running_up_      = true;
    runup_start_pos_ = pos;
    breakaway_ms_    = millis();
    setSpeed(static_cast<int16_t>(-turn_dir_ * turn_cruise_));
}

bool MotorHAL::updateBreakaway(long pos) {
    const uint32_t now = millis();

    // Anlauf rueckwaerts: bis RUNUP_COUNTS geschafft sind - oder RUNUP_MAX_MS
    // um sind (dann steht er rueckwaerts an einem Anschlag). Danach in
    // Fahrtrichtung und die Fortschrittsmessung neu beginnen.
    if (running_up_) {
        const long back = -turn_dir_ * (pos - runup_start_pos_);
        if (back < RUNUP_COUNTS && now - breakaway_ms_ < RUNUP_MAX_MS) return true;
        running_up_   = false;
        breakaway_ms_ = now;
        turn_ref_pos_ = pos;
        turn_ref_ms_  = now;
        setSpeed(static_cast<int16_t>(turn_dir_ * turn_cruise_));
        return true;
    }

    // Bewegt er sich in Fahrtrichtung? Dann ab hier die normale Ueberwachung.
    if (turn_dir_ * (pos - turn_ref_pos_) >= TURN_MIN_COUNTS) {
        turn_moving_ = true;
        if (breakaway_tries_ > 1) {
            Serial.printf("INFO  [MotorHAL]: Motor %u erst im %u. Anlauf losgekommen.\n",
                          motor_id_, breakaway_tries_);
        }
        return false;
    }

    if (now - breakaway_ms_ < BREAKAWAY_DETECT_MS) return true;

    if (breakaway_tries_ >= BREAKAWAY_TRIES) {
        off();
        turn_failed_ = true;
        Serial.printf("WARN  [MotorHAL]: Motor %u laeuft nicht an - %u Anlaeufe ohne "
                      "Erfolg, Fahrt abgebrochen (Position %ld, Ziel %ld).\n",
                      motor_id_, BREAKAWAY_TRIES, pos, turn_target_);
        Serial.println("WARN  [MotorHAL]: Mechanik fest, Strombegrenzung (ILIM) zu "
                       "niedrig, Versorgung bricht ein, oder Encoder zaehlt nicht.");
        return true;
    }

    Serial.printf("INFO  [MotorHAL]: Motor %u steht - Anlauf %u/%u.\n",
                  motor_id_, breakaway_tries_ + 1, BREAKAWAY_TRIES);
    startRunup(pos);
    return true;
}

void MotorHAL::updateTurn() {
    if (!turning_ || !enc_) return;

    // Bis zur ersten Bewegung uebernimmt das Losbrechen die Ueberwachung.
    if (!turn_moving_ && updateBreakaway(enc_->read())) return;

    // Vergleich in Fahrtrichtung, nicht ueber den Betrag: Ein simples
    // labs(pos - target) <= Toleranz wuerde bei zu grosser Schrittweite
    // zwischen zwei Loops uebersprungen und die Drehung liefe endlos weiter.
    const long pos       = enc_->read();
    const bool forward   = current_speed_ > 0;
    const bool reached   = forward ? pos >= turn_target_ : pos <= turn_target_;
    const long remaining = labs(turn_target_ - pos);
    const bool in_window = soft_approach_ && remaining <= SOFT_APPROACH_WINDOW_COUNTS;

    // Kurz vor dem Ziel auf SOFT_APPROACH_SPEED abbremsen - weniger
    // Aufprallenergie an einem mechanischen Anschlag. Nur einmalig aufrufen
    // (sonst schreibt writeChannels() bei jedem Loop unnoetig denselben Wert
    // erneut).
    if (in_window && abs(current_speed_) > SOFT_APPROACH_SPEED) {
        setSpeed(forward ? SOFT_APPROACH_SPEED : static_cast<int16_t>(-SOFT_APPROACH_SPEED));
    }

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
            return;
        }

        // In der Anfahrzone (in_window) gilt ein VIEL kuerzeres Stall-Fenster:
        // Ein Motor, der dort mit vollem TURN_SPEED gegen einen Anschlag steht,
        // kann per Blockierstrom die Versorgung einbrechen lassen (siehe
        // SOFT_APPROACH_* oben) - das Fenster ist bewusst kurz, um genau das
        // zu verhindern.
        const uint32_t stall_ms = in_window ? SOFT_APPROACH_STALL_MS : TURN_STALL_MS;
        if (millis() - turn_ref_ms_ < stall_ms) return;

        off();

        if (in_window) {
            // KEIN Fehler: In der Anfahrzone ist "kein Fortschritt mehr" das
            // ERWARTETE Ergebnis - Kontakt/Anschlag statt Encoder-Ziel. Deshalb
            // bleibt turn_failed_ false.
            Serial.printf(
                "INFO  [MotorHAL]: Sanfte Anfahrt beendet (Kontakt/Anschlag) - "
                "Position %ld (Ziel %ld, Rest %ld Counts).\n",
                pos, turn_target_, remaining);
            return;
        }

        turn_failed_ = true;
        Serial.printf(
            "WARN  [MotorHAL]: Drehung abgebrochen - in %lu ms nur %ld Counts "
            "in Fahrtrichtung. Position %ld (Start %ld, Ziel %ld, PWM %d).\n",
            static_cast<unsigned long>(stall_ms), progress,
            pos, turn_start_, turn_target_, current_speed_);
        Serial.println(
            "WARN  [MotorHAL]: Position unveraendert -> Encoder zaehlt nicht "
            "(Verdrahtung/Pins pruefen). Position laeuft weg -> Encoder-Kanaele "
            "A/B vertauscht. Mechanik fest -> Motor blockiert.");
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
