#include "hal/rexus_hal.hpp"

REXUSHAL::REXUSHAL(uint8_t pin_l0_t, uint8_t pin_soe_i, uint8_t pin_sods_i)
    : pin_l0_t_(pin_l0_t), pin_soe_i_(pin_soe_i), pin_sods_i_(pin_sods_i) {
}

bool REXUSHAL::init() {
    // Die Leitungen liegen am Bodenaufbau offen. Der interne Pull zieht sie
    // deshalb IMMER auf den Ruhepegel - also entgegengesetzt zum aktiven
    // Pegel. Ohne das würde eingekoppeltes Rauschen als SODS/LO/SOE gelesen
    // und die Flugsequenz auslösen.
    const uint8_t mode = REXUS_ACTIVE_HIGH ? INPUT_PULLDOWN : INPUT_PULLUP;
    pinMode(pin_l0_t_, mode);
    pinMode(pin_soe_i_, mode);
    pinMode(pin_sods_i_, mode);

    // Dem Pull Zeit geben, die Leitungskapazitaet umzuladen - sonst wird noch
    // der Pegel von vor dem pinMode eingelesen.
    delayMicroseconds(500);

    const uint32_t now = millis();
    seed(l0_,   pin_l0_t_,   now);
    seed(soe_,  pin_soe_i_,  now);
    seed(sods_, pin_sods_i_, now);

    initialized_ = true;
    return true;
}

void REXUSHAL::seed(Debounced& d, uint8_t pin, uint32_t now_ms) {
    const bool level = (digitalRead(pin) == HIGH);
    d.raw      = level;
    d.last_raw = level;
    d.stable   = level;
    d.since_ms = now_ms;
}

void REXUSHAL::sample(Debounced& d, bool raw, uint32_t now_ms) {
    d.raw = raw;

    if (raw != d.last_raw) {          // Flanke -> Stabilitaetsfenster neu starten
        d.last_raw = raw;
        d.since_ms = now_ms;
        return;
    }
    if (raw != d.stable && (now_ms - d.since_ms) >= SIGNAL_STABLE_MS) {
        d.stable = raw;               // lange genug stabil -> uebernehmen
    }
}

void REXUSHAL::update(uint32_t now_ms) {
    if (!initialized_) return;
    // raw = elektrischer Pegel (fuer rawBits), stable = interpretiertes Signal.
    sample(l0_,   digitalRead(pin_l0_t_)   == HIGH, now_ms);
    sample(soe_,  digitalRead(pin_soe_i_)  == HIGH, now_ms);
    sample(sods_, digitalRead(pin_sods_i_) == HIGH, now_ms);
}

/// Rechnet einen entprellten PIN-Pegel in "Signal liegt an" um.
static inline bool asserted(bool level) {
    return REXUS_ACTIVE_HIGH ? level : !level;
}

uint8_t REXUSHAL::rawBits() const {
    uint8_t bits = 0;
    if (l0_.raw)   bits |= DL_REXUS_L0;
    if (soe_.raw)  bits |= DL_REXUS_SOE;
    if (sods_.raw) bits |= DL_REXUS_SODS;
    return bits;
}

// Die Leser liefern den ENTPRELLTEN Pegel - Quelle ist update(), nicht der Pin.
bool REXUSHAL::readL0T() {
    if (!initialized_) return false;
    return asserted(l0_.stable);
}

bool REXUSHAL::readSOE() {
    if (!initialized_) return false;
    return asserted(soe_.stable);
}

bool REXUSHAL::readSODS() {
    if (!initialized_) return false;
    return asserted(sods_.stable);
}

void REXUSHAL::getAllSignals(bool& l0_t, bool& soe, bool& sods) {
    if (!initialized_) {
        l0_t = false;
        soe = false;
        sods = false;
        return;
    }
    
    l0_t = readL0T();
    soe = readSOE();
    sods = readSODS();
}

bool REXUSHAL::isLaunched() {
    return readL0T();
}

bool REXUSHAL::isExperimentActive() {
    return readSOE();
}

bool REXUSHAL::isDescending() {
    return readSODS();
}
