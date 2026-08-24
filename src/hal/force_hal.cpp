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

    tare();
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
    if (!ready()) return false;

    int32_t raw[FORCE_MAX_CHANNELS];
    shiftOut24(raw);
    last_sample_ms_ = millis();

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

bool ForceHAL::tare(uint8_t samples, uint32_t timeout_ms) {
    if (!initialized_) return false;
    if (samples == 0) samples = 1;

    // int64 als Summe: 8 Samples x 24 Bit passen zwar auch in int32, aber der
    // Aufrufer darf samples erhoehen, ohne dass es hier still ueberlaeuft.
    int64_t sum[FORCE_MAX_CHANNELS] = {};
    uint8_t got = 0;

    const uint32_t start = millis();
    while (got < samples && millis() - start < timeout_ms) {
        if (!ready()) continue;

        int32_t raw[FORCE_MAX_CHANNELS];
        shiftOut24(raw);
        last_sample_ms_ = millis();

        for (uint8_t i = 0; i < count_; i++) sum[i] += raw[i];
        got++;
    }

    if (got == 0) {
        // Kein Wandler hat geantwortet - alten Nullpunkt NICHT ueberschreiben.
        tared_ = false;
        return false;
    }

    for (uint8_t i = 0; i < count_; i++) {
        offset_[i] = static_cast<int32_t>(sum[i] / got);
    }
    tared_ = true;
    return true;
}
