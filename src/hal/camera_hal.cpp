#include "hal/camera_hal.hpp"

namespace {
constexpr uint8_t CMD_GET_DEVICE_INFO = 0x00;
constexpr uint8_t CMD_CAMERA_CONTROL  = 0x01;
constexpr uint8_t CTRL_START_RECORDING = 0x03;
constexpr uint8_t CTRL_STOP_RECORDING  = 0x04;
constexpr uint8_t DEVICE_INFO_LEN      = 5;   // 0xCC | Version | Features LE | CRC

/// Signed-Vergleich - uebersteht den millis()-Ueberlauf.
inline bool reached(uint32_t now, uint32_t t) { return static_cast<int32_t>(now - t) >= 0; }
}  // namespace

CameraHAL::CameraHAL(HardwareSerial& serial, uint8_t pin_sel_a, uint8_t pin_sel_b)
    : serial_(serial), pin_a_(pin_sel_a), pin_b_(pin_sel_b) {}

bool CameraHAL::init(uint32_t baudrate) {
    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);
    select(0);
    serial_.begin(baudrate);
    initialized_ = true;
    next_probe_ms_ = millis() + CAM_PROBE_PERIOD_MS;
    // Gleich nach dem Start einmal alle abfragen - zeigt am Boden sofort,
    // welche Kameras an sind.
    for (uint8_t c = 0; c < CAM_COUNT; ++c) pending_probe_[c] = true;
    return true;
}

uint8_t CameraHAL::crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

void CameraHAL::request(uint8_t cam, CamAction action) {
    if (cam >= CAM_COUNT) return;
    if (action == CamAction::PROBE) pending_probe_[cam] = true;
    else if (action == CamAction::START || action == CamAction::STOP) pending_cmd_[cam] = action;
}

void CameraHAL::requestAll(CamAction action) {
    for (uint8_t c = 0; c < CAM_COUNT; ++c) request(c, action);
}

bool CameraHAL::alive(uint8_t cam, uint32_t now_ms) const {
    if (cam >= CAM_COUNT || !status_[cam].answered_ever) return false;
    return now_ms - status_[cam].last_answer_ms < CAM_ALIVE_MS;
}

void CameraHAL::select(uint8_t channel) {
    // Kanal = A + 2*B (CD4052B-Wahrheitstabelle, siehe camera_hal.hpp)
    digitalWrite(pin_a_, (channel & 0x01) ? HIGH : LOW);
    digitalWrite(pin_b_, (channel & 0x02) ? HIGH : LOW);
    channel_ = channel;
}

void CameraHAL::drainRx() {
    while (serial_.available() > 0) serial_.read();
    rx_len_ = 0;
}

bool CameraHAL::pickJob(uint32_t now_ms) {
    // 1. Steuerbefehle (START/STOP) zuerst, reihum ueber die Kameras.
    for (uint8_t i = 0; i < CAM_COUNT; ++i) {
        const uint8_t c = (next_cmd_cam_ + i) % CAM_COUNT;
        if (pending_cmd_[c] != CamAction::NONE) {
            job_cam_ = c;
            job_ = pending_cmd_[c];
            pending_cmd_[c] = CamAction::NONE;
            next_cmd_cam_ = (c + 1) % CAM_COUNT;
            return true;
        }
    }
    // 2. Angeforderte Abfragen.
    for (uint8_t c = 0; c < CAM_COUNT; ++c) {
        if (pending_probe_[c]) {
            pending_probe_[c] = false;
            job_cam_ = c;
            job_ = CamAction::PROBE;
            return true;
        }
    }
    // 3. Regelmaessige Abfrage, eine Kamera pro Periode.
    if (reached(now_ms, next_probe_ms_)) {
        next_probe_ms_ = now_ms + CAM_PROBE_PERIOD_MS;
        job_cam_ = next_probe_cam_;
        job_ = CamAction::PROBE;
        next_probe_cam_ = (next_probe_cam_ + 1) % CAM_COUNT;
        return true;
    }
    return false;
}

bool CameraHAL::sendJob() {
    uint8_t buf[4];
    uint8_t len = 0;
    buf[len++] = CAM_HEADER;
    if (job_ == CamAction::PROBE) {
        buf[len++] = CMD_GET_DEVICE_INFO;
    } else {
        buf[len++] = CMD_CAMERA_CONTROL;
        buf[len++] = (job_ == CamAction::START) ? CTRL_START_RECORDING : CTRL_STOP_RECORDING;
    }
    buf[len] = crc8(buf, len);
    ++len;
    if (serial_.availableForWrite() < len) return false;   // naechster Loop
    serial_.write(buf, len);
    return true;
}

bool CameraHAL::readResponse(uint32_t now_ms) {
    while (serial_.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(serial_.read());
        ++job_rx_bytes_;
        status_[job_cam_].rx_any = true;
        if (rx_len_ == 0 && b != CAM_HEADER) continue;      // auf den Kopf warten
        rx_[rx_len_++] = b;
        if (rx_len_ < DEVICE_INFO_LEN) continue;

        if (crc8(rx_, DEVICE_INFO_LEN - 1) == rx_[DEVICE_INFO_LEN - 1]) {
            CameraStatus& s = status_[job_cam_];
            const bool first = !s.answered_ever;
            s.answered_ever  = true;
            s.last_answer_ms = now_ms;
            s.protocol       = rx_[1];
            s.features       = static_cast<uint16_t>(rx_[2] | (rx_[3] << 8));
            if (first) {
                Serial.printf("INFO  [Kamera]: Kamera %u antwortet (Protokoll %u, Features 0x%04X%s).\n",
                              job_cam_ + 1, s.protocol, s.features,
                              (s.features & CAM_FEATURE_START_RECORDING) ? "" : ", KEIN START_RECORDING");
            }
            return true;
        }
        // CRC falsch: ab dem naechsten 0xCC im Puffer neu suchen.
        uint8_t k = 1;
        while (k < rx_len_ && rx_[k] != CAM_HEADER) ++k;
        for (uint8_t i = k; i < rx_len_; ++i) rx_[i - k] = rx_[i];
        rx_len_ -= k;
    }
    return false;
}

void CameraHAL::update(uint32_t now_ms) {
    if (!initialized_) return;

    switch (phase_) {
        case Phase::IDLE:
            if (!pickJob(now_ms)) return;
            if (channel_ != job_cam_) select(job_cam_);
            phase_until_ = now_ms + CAM_SETTLE_MS;
            phase_ = Phase::SETTLE;
            return;

        case Phase::SETTLE:
            if (!reached(now_ms, phase_until_)) return;
            drainRx();   // Stoerbytes vom Umschalten und Reste der vorigen Kamera
            if (!sendJob()) return;
            if (job_ == CamAction::PROBE) {
                job_rx_bytes_ = 0;
                phase_until_ = now_ms + CAM_RESPONSE_TIMEOUT_MS;
                phase_ = Phase::WAIT_RESPONSE;
            } else {
                status_[job_cam_].last_cmd = job_;
                Serial.printf("INFO  [Kamera]: Kamera %u: %s gesendet.\n", job_cam_ + 1,
                              job_ == CamAction::START ? "START_RECORDING" : "STOP_RECORDING");
                phase_until_ = now_ms + CAM_TX_GAP_MS;
                phase_ = Phase::GAP;
            }
            return;

        case Phase::WAIT_RESPONSE:
            if (readResponse(now_ms)) {
                phase_until_ = now_ms + CAM_TX_GAP_MS;
                phase_ = Phase::GAP;
            } else if (reached(now_ms, phase_until_)) {
                ++status_[job_cam_].probes_failed;
                if (job_rx_bytes_ > 0) status_[job_cam_].rx_garbage = true;
                phase_until_ = now_ms;
                phase_ = Phase::GAP;
            }
            return;

        case Phase::GAP:
            if (reached(now_ms, phase_until_)) phase_ = Phase::IDLE;
            return;
    }
}
