#include "hal/camera_hal.hpp"
#include "obc_log.hpp"

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
    if (cam >= CAM_COUNT) return false;
    const CameraStatus& s = status_[cam];
    const bool answer = s.answered_ever &&
        static_cast<int32_t>(now_ms - s.last_answer_ms) < static_cast<int32_t>(CAM_ALIVE_MS);
    const bool bytes  = s.rx_any &&
        static_cast<int32_t>(now_ms - s.last_rx_ms) < static_cast<int32_t>(CAM_ALIVE_MS);
    return answer || bytes;
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
        if (diag_len_ < DIAG_BYTES) diag_[diag_len_++] = b;
        status_[job_cam_].rx_any = true;
        status_[job_cam_].last_rx_ms = now_ms;
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
                Log.printf("INFO  [Kamera]: Kamera %u antwortet (Protokoll %u, Features 0x%04X%s).\n",
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

void CameraHAL::logFailedProbe(uint32_t now_ms) {
    if (diag_logged_ >= DIAG_FREE_LOGS &&
        static_cast<int32_t>(now_ms - diag_last_ms_) < static_cast<int32_t>(DIAG_INTERVAL_MS)) return;
    if (diag_logged_ < 0xFF) diag_logged_++;
    diag_last_ms_ = now_ms;
    char hex[3 * DIAG_BYTES + 1];
    hex[0] = '\0';
    for (uint8_t i = 0; i < diag_len_; i++) snprintf(&hex[3 * i], 4, "%02X ", diag_[i]);
    Log.printf("INFO  [Kamera]: Kamera %u keine gueltige Antwort in %lu ms - %u Byte empfangen: %s\n",
               job_cam_ + 1, static_cast<unsigned long>(now_ms - job_start_ms_),
               job_rx_bytes_, diag_len_ ? hex : "(nichts)");
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
                diag_len_ = 0;
                job_start_ms_ = now_ms;
                phase_until_ = now_ms + CAM_RESPONSE_TIMEOUT_MS;
                phase_ = Phase::WAIT_RESPONSE;
            } else {
                status_[job_cam_].last_cmd = job_;
                Log.printf("INFO  [Kamera]: Kamera %u: %s gesendet.\n", job_cam_ + 1,
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
                logFailedProbe(now_ms);
                phase_until_ = now_ms;
                phase_ = Phase::GAP;
            }
            return;

        case Phase::GAP:
            if (reached(now_ms, phase_until_)) phase_ = Phase::IDLE;
            return;
    }
}
