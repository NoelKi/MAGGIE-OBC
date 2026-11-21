#include "obc_log.hpp"

ObcLog Log;

size_t ObcLog::write(uint8_t b) {
    Serial.write(b);
    collect(b);
    return 1;
}

size_t ObcLog::write(const uint8_t* buffer, size_t size) {
    Serial.write(buffer, size);
    for (size_t i = 0; i < size; i++) collect(buffer[i]);
    return size;
}

void ObcLog::collect(uint8_t b) {
    if (b == '\r') return;
    if (b == '\n') { emit(); return; }
    line_[len_++] = static_cast<char>(b);
    if (len_ >= LINE_MAX) emit();
}

void ObcLog::emit() {
    if (sink_) sink_(ctx_, line_, len_);
    len_ = 0;
}
