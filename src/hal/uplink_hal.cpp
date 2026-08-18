#include "hal/uplink_hal.hpp"

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

bool UplinkReceiver::poll(MotorCommand& cmd) {
    while (serial_.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(serial_.read());

        if (len_ == 0) {
            // Auf Start-Marker synchronisieren.
            if (b != UL_START) continue;
            buf_[len_++] = b;
            continue;
        }

        buf_[len_++] = b;
        if (len_ < UPLINK_FRAME_SIZE) continue;

        // Vollständiges Frame-Kandidat -> validieren.
        len_ = 0;

        if (buf_[UPLINK_FRAME_SIZE - 1] != UL_END) {
            // Kein sauberes Frame; falls das letzte Byte selbst ein START ist,
            // hätte der nächste poll()-Durchlauf ohnehin damit begonnen -> hier
            // schlicht verwerfen und neu synchronisieren.
            continue;
        }
        if (crc8(&buf_[1], 3) != buf_[4]) {
            continue;   // CRC-Fehler -> verwerfen
        }

        cmd.opcode = buf_[1];
        cmd.arg    = static_cast<int16_t>((buf_[2] << 8) | buf_[3]);
        cmd.valid  = true;
        return true;
    }
    return false;
}
