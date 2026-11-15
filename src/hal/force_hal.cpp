#include "hal/force_hal.hpp"

ForceHAL::ForceHAL(const uint8_t* dout_pins, uint8_t count, uint8_t pin_sck, int32_t tele_div)
    : pin_sck_(pin_sck), tele_div_(tele_div) {
    if (count > FORCE_MAX_CHANNELS) count = FORCE_MAX_CHANNELS;
    count_ = count;
    for (uint8_t i = 0; i < count_; i++) dout_[i] = dout_pins[i];
}

bool ForceHAL::init() {
    // INPUT_PULLUP statt INPUT: Der HX711 treibt DOUT aktiv, der Pull-up stoert
    // ihn also nicht. Fehlt ein Wandler oder ist die Leitung ab, liegt der Pin
    // damit auf HIGH = "noch nicht fertig" - ready() wird nie wahr und der
    // Sensor meldet sich sauber als haengend ab. Mit reinem INPUT wuerde der
    // offene Pin zufaellig LOW lesen und Rauschen als Messwert ausgeben.
    for (uint8_t i = 0; i < count_; i++) pinMode(dout_[i], INPUT_PULLUP);

    pinMode(pin_sck_, OUTPUT);
    digitalWrite(pin_sck_, LOW);   // HIGH wuerde die Wandler schlafen legen

    initialized_    = true;
    last_sample_ms_ = millis();

    startTare();
    return true;
}

bool ForceHAL::ready() const {
    if (!initialized_) return false;
    // DOUT LOW = Wandlung fertig. Nur wenn ALLE so weit sind, darf getaktet
    // werden - der gemeinsame Takt trifft sonst einen Wandler mitten in der
    // Wandlung.
    for (uint8_t i = 0; i < count_; i++) {
        if (digitalRead(dout_[i]) != LOW) return false;
    }
    return true;
}

bool ForceHAL::stalled() const {
    return millis() - last_sample_ms_ > FORCE_STALE_MS;
}

int32_t ForceHAL::signExtend24(uint32_t value) {
    // Bit 23 ist das Vorzeichen des 24-Bit-Zweierkomplements.
    if (value & 0x00800000UL) value |= 0xFF000000UL;
    return static_cast<int32_t>(value);
}

int16_t ForceHAL::toTelemetry(int32_t counts, int32_t tele_div, bool& saturated) {
    const int32_t scaled = counts / tele_div;
    if (scaled > 32767) {
        saturated = true;
        return 32767;
    }
    if (scaled < -32768) {
        saturated = true;
        return -32768;
    }
    saturated = false;
    return static_cast<int16_t>(scaled);
}

void ForceHAL::shiftOut24(int32_t* out) {
    uint32_t acc[FORCE_MAX_CHANNELS] = {};

    // 24 Datenbits, MSB zuerst. Gelesen wird waehrend SCK HIGH: Der Wandler
    // legt das Bit nach der fallenden Flanke an, es steht also ueber die
    // naechste steigende Flanke hinweg stabil.
    for (uint8_t bit = 0; bit < 24; bit++) {
        digitalWrite(pin_sck_, HIGH);
        delayMicroseconds(1);

        // Alle Leitungen der Gruppe im selben Taktschritt - das ist der ganze
        // Grund fuer diesen Treiber (siehe force_hal.hpp).
        for (uint8_t i = 0; i < count_; i++) {
            acc[i] = (acc[i] << 1) | static_cast<uint32_t>(digitalRead(dout_[i]));
        }

        digitalWrite(pin_sck_, LOW);
        delayMicroseconds(1);
    }

    // 25. Impuls: waehlt fuer die naechste Wandlung Kanal A mit Gain 128.
    // Ohne ihn bliebe DOUT LOW und der Wandler startet keine neue Messung.
    digitalWrite(pin_sck_, HIGH);
    delayMicroseconds(1);
    digitalWrite(pin_sck_, LOW);
    delayMicroseconds(1);

    for (uint8_t i = 0; i < count_; i++) out[i] = signExtend24(acc[i]);
}

bool ForceHAL::read(ForceReading& out) {
    // Timeout des Nullabgleichs bei JEDEM Aufruf pruefen, nicht nur bei einem
    // neuen Sample: Antwortet kein Wandler, kommt hier nie eines an.
    if (tare_running_ && millis() - tare_start_ms_ >= tare_timeout_ms_) {
        finishTare();
    }

    if (!ready()) return false;

    int32_t raw[FORCE_MAX_CHANNELS];
    shiftOut24(raw);
    last_sample_ms_ = millis();

    if (tare_running_) {
        for (uint8_t i = 0; i < count_; i++) tare_sum_[i] += raw[i];
        if (++tare_got_ >= tare_want_) finishTare();
    }

    out.count = count_;
    for (uint8_t i = 0; i < count_; i++) {
        out.raw[i]    = raw[i];
        out.counts[i] = raw[i] - offset_[i];
        out.tele[i]   = toTelemetry(out.counts[i], tele_div_, out.sat[i]);
    }

    out.timestamp = last_sample_ms_;
    out.valid     = true;
    return true;
}

bool ForceHAL::startTare(uint8_t samples, uint32_t timeout_ms) {
    if (!initialized_) return false;

    tare_want_       = samples == 0 ? 1 : samples;
    tare_got_        = 0;
    tare_start_ms_   = millis();
    tare_timeout_ms_ = timeout_ms;
    for (uint8_t i = 0; i < FORCE_MAX_CHANNELS; i++) tare_sum_[i] = 0;
    tare_done_    = false;
    tare_running_ = true;
    return true;
}

void ForceHAL::finishTare() {
    tare_running_ = false;

    if (tare_got_ == 0) {
        // Kein Wandler hat geantwortet - alten Nullpunkt NICHT ueberschreiben.
        tared_    = false;
        tare_ok_  = false;
        tare_done_ = true;
        return;
    }

    // int64 als Summe: 8 Samples x 24 Bit passen zwar auch in int32, aber der
    // Aufrufer darf samples erhoehen, ohne dass es hier still ueberlaeuft.
    for (uint8_t i = 0; i < count_; i++) {
        offset_[i] = static_cast<int32_t>(tare_sum_[i] / tare_got_);
    }
    tared_     = true;
    tare_ok_   = true;
    tare_done_ = true;
}
