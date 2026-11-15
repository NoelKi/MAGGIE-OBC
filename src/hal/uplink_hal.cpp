#include "hal/uplink_hal.hpp"
#include <cstring>

UplinkReceiver::UplinkReceiver(HardwareSerial& serial)
    : serial_(serial) {
}

uint8_t UplinkReceiver::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }
    return crc;
}

bool UplinkReceiver::poll(UplinkCommand& cmd) {
    while (serial_.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(serial_.read());
        rx_bytes_++;    // zaehlt JEDES Byte, auch die RXSM-Huelle davor

        // Burst-Erkennung: nach einer Sendepause faengt ein neues Paket an.
        // Nur so zeigt der Mitschnitt die Kopfbytes statt des Paketendes.
        const uint32_t now = millis();
        if (now - last_byte_ms_ > BURST_GAP_MS) burst_len_ = 0;
        last_byte_ms_ = now;
        if (burst_len_ < BURST_CAPTURE) burst_[burst_len_++] = b;

        // Auf den Start-Marker synchronisieren.
        if (len_ == 0 && b != UL_START) continue;

        buf_[len_++] = b;
        if (len_ < UPLINK_FRAME_SIZE) continue;

        // Vollständiger Frame-Kandidat -> validieren.
        if (buf_[UPLINK_FRAME_SIZE - 1] == UL_END && crc8(&buf_[1], 3) == buf_[4]) {
            cmd.opcode = buf_[1];
            cmd.arg    = static_cast<int16_t>((buf_[2] << 8) | buf_[3]);
            cmd.valid  = true;
            len_ = 0;
            frames_ok_++;
            last_opcode_ = cmd.opcode;
            return true;
        }

        frames_bad_++;
        resync();
    }
    return false;
}

void UplinkReceiver::resync() {
    // Der Kandidat war kein gültiges Frame (falscher End-Marker oder CRC).
    // Das erste Byte war also ein Fehlstart: im bereits gelesenen Rest nach
    // dem nächsten Start-Marker suchen und dort neu aufsetzen. Ohne diesen
    // Schritt würde ein 0x7E im Störbyte-Vorlauf das unmittelbar folgende
    // gültige Frame mit verschlucken.
    for (uint8_t i = 1; i < UPLINK_FRAME_SIZE; i++) {
        if (buf_[i] != UL_START) continue;
        len_ = static_cast<uint8_t>(UPLINK_FRAME_SIZE - i);
        memmove(buf_, &buf_[i], len_);
        return;
    }
    len_ = 0;   // kein weiterer Startmarker im Puffer -> alles verwerfen
}
