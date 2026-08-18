#include "hal/camera_bus.hpp"

CameraBus::CameraBus(HardwareSerial* serial, uint8_t select_a_pin, uint8_t select_b_pin)
    : serial_(serial),
      select_a_pin_(select_a_pin),
      select_b_pin_(select_b_pin) {
}

bool CameraBus::begin(uint32_t baudrate) {
    if (!serial_) {
        initialized_ = false;
        return false;
    }

    pinMode(select_a_pin_, OUTPUT);
    pinMode(select_b_pin_, OUTPUT);
    digitalWrite(select_a_pin_, LOW);
    digitalWrite(select_b_pin_, LOW);

    serial_->begin(baudrate);

    initialized_ = true;
    current_channel_ = NO_CHANNEL;  // erzwingt echtes Umschalten beim ersten select()
    return select(0);
}

bool CameraBus::select(uint8_t channel) {
    if (!initialized_ || channel >= MAX_CHANNELS) return false;
    if (channel == current_channel_) return true;

    // Erst die laufende Übertragung zu Ende bringen. Wird der Mux mitten in
    // einem Byte umgeschaltet, bekommt die alte Kamera ein abgeschnittenes
    // Kommando und die neue ein Bruchstück - beides kann als gültiges Byte
    // durchgehen und ein ungewolltes Toggle auslösen.
    serial_->flush();

    digitalWrite(select_a_pin_, (channel & 0x01) ? HIGH : LOW);
    digitalWrite(select_b_pin_, (channel & 0x02) ? HIGH : LOW);
    delayMicroseconds(SETTLE_US);

    current_channel_ = channel;

    // Was während des Umschaltens in den Empfangspuffer gerutscht ist, lässt
    // sich keiner Kamera mehr eindeutig zuordnen.
    flushInput();
    return true;
}

void CameraBus::flushInput() {
    if (!serial_) return;
    while (serial_->available() > 0) {
        (void)serial_->read();
    }
}
