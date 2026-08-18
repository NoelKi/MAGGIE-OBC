#include "hal/camera_hal.hpp"

CameraHAL::CameraHAL(const CameraConfig& config)
    : camera_id_(config.camera_id),
      serial_(config.serial),
      baudrate_(config.baudrate),
      model_(config.model),
      mode_(config.mode),
      boot_delay_ms_(config.boot_delay_ms) {
}

bool CameraHAL::init() {
    if (!serial_) {
        // Kein UART-Port konfiguriert (siehe camera_config.hpp) - Kamera bleibt inaktiv.
        state_ = CameraState::UNCONFIGURED;
        return false;
    }

    serial_->begin(baudrate_);

    // Die Kamera braucht nach dem Einschalten mehrere Sekunden, bis ihr UART
    // antwortet. Vorher gesendete Kommandos gehen verloren, deshalb erst nach
    // boot_delay_ms_ mit dem Handshake beginnen.
    init_ms_ = millis();
    state_ = CameraState::BOOTING;
    return true;
}

void CameraHAL::update(uint32_t now_ms) {
    switch (state_) {
        case CameraState::UNCONFIGURED:
            return;

        case CameraState::BOOTING:
            if ((now_ms - init_ms_) >= boot_delay_ms_) {
                requestDeviceInfo(now_ms);
                state_ = CameraState::DETECTING;
            }
            return;

        case CameraState::DETECTING:
            pollDeviceInfo(now_ms);
            if (state_ != CameraState::READY) return;
            break;  // Im selben Durchlauf direkt weiter zum Auto-Trigger

        case CameraState::READY:
            break;
    }

    // Ab hier: state_ == READY
    if (mode_ == CameraTriggerMode::RECORD_ON_BOOT && !auto_record_done_) {
        // Erst als erledigt markieren, wenn das Kommando wirklich rausging -
        // sonst geht die Aufnahme verloren, falls das Rate-Limit gerade greift.
        auto_record_done_ = startRecording();
    }
}

void CameraHAL::requestDeviceInfo(uint32_t now_ms) {
    rx_count_ = 0;
    last_probe_ms_ = now_ms;
    ++probe_attempts_;

    // Alte/unvollständige Bytes aus dem Empfangspuffer werfen, damit die
    // Antwort nicht durch Reste eines vorherigen Versuchs verschoben wird.
    while (serial_->available() > 0) {
        (void)serial_->read();
    }

    sendCommand(RunCam::Cmd::GET_DEVICE_INFO, nullptr, 0);
}

void CameraHAL::pollDeviceInfo(uint32_t now_ms) {
    while (serial_->available() > 0 && rx_count_ < RunCam::DEVICE_INFO_RESPONSE_LENGTH) {
        const int c = serial_->read();
        if (c < 0) break;

        // Auf den Header synchronisieren: alles vor 0xCC ist Müll.
        if (rx_count_ == 0 && static_cast<uint8_t>(c) != RunCam::HEADER) continue;

        rx_buffer_[rx_count_++] = static_cast<uint8_t>(c);
    }

    if (rx_count_ == RunCam::DEVICE_INFO_RESPONSE_LENGTH) {
        // CRC läuft über alle Bytes inkl. Header, das letzte Byte ist der CRC selbst.
        const uint8_t crc = RunCam::crc8DvbS2Buffer(rx_buffer_,
                                                    RunCam::DEVICE_INFO_RESPONSE_LENGTH - 1);
        if (crc == rx_buffer_[RunCam::DEVICE_INFO_RESPONSE_LENGTH - 1]) {
            protocol_version_ = rx_buffer_[1];
            features_ = static_cast<uint16_t>(rx_buffer_[2]) |
                        (static_cast<uint16_t>(rx_buffer_[3]) << 8);
            device_detected_ = true;
            state_ = CameraState::READY;
            return;
        }
        // CRC falsch -> Antwort verwerfen und neu anfragen.
        rx_count_ = 0;
    }

    if ((now_ms - last_probe_ms_) < PROBE_INTERVAL_MS) return;

    if (probe_attempts_ >= MAX_PROBE_ATTEMPTS) {
        // Keine Antwort. Das kann an einer fehlenden/defekten RX-Leitung liegen,
        // während TX zur Kamera funktioniert. Für einen einmaligen Flug ist es
        // besser, die Kommandos trotzdem blind zu senden, als die Aufnahme zu
        // verlieren. device_detected_ bleibt false und geht so in die Telemetrie.
        state_ = CameraState::READY;
        return;
    }

    requestDeviceInfo(now_ms);
}

bool CameraHAL::sendCommand(uint8_t command, const uint8_t* payload, size_t payload_length) {
    if (!serial_ || state_ == CameraState::UNCONFIGURED) return false;
    if (payload_length + 3 > RunCam::MAX_TX_PACKET_LENGTH) return false;

    uint8_t packet[RunCam::MAX_TX_PACKET_LENGTH];
    size_t length = 0;

    packet[length++] = RunCam::HEADER;
    packet[length++] = command;
    for (size_t i = 0; i < payload_length; ++i) {
        packet[length++] = payload[i];
    }
    packet[length] = RunCam::crc8DvbS2Buffer(packet, length);
    ++length;

    return serial_->write(packet, length) == length;
}

bool CameraHAL::sendControlAction(uint8_t action) {
    if (state_ != CameraState::READY) return false;

    // Die Split 4 schaltet die Aufnahme per Toggle. Kommen zwei Kommandos zu
    // schnell hintereinander, hebt das zweite das erste wieder auf.
    const uint32_t now_ms = millis();
    if (last_control_ms_ != 0 && (now_ms - last_control_ms_) < MIN_CONTROL_INTERVAL_MS) {
        return false;
    }

    if (!sendCommand(RunCam::Cmd::CAMERA_CONTROL, &action, 1)) return false;

    last_control_ms_ = now_ms;
    return true;
}

bool CameraHAL::startRecording() {
    if (recording_) return true;

    // Split 4: SIMULATE_POWER_BTN ist ein Toggle, START_RECORDING wird ignoriert.
    const uint8_t action = (model_ == RunCamModel::SPLIT_4)
                               ? RunCam::Action::SIMULATE_POWER_BTN
                               : RunCam::Action::START_RECORDING;

    if (!sendControlAction(action)) return false;

    recording_ = true;
    return true;
}

bool CameraHAL::stopRecording() {
    if (!recording_) return true;

    const uint8_t action = (model_ == RunCamModel::SPLIT_4)
                               ? RunCam::Action::SIMULATE_POWER_BTN
                               : RunCam::Action::STOP_RECORDING;

    if (!sendControlAction(action)) return false;

    recording_ = false;
    return true;
}

bool CameraHAL::changeMode() {
    // Nur im Standby sinnvoll - ein Moduswechsel während der Aufnahme würde
    // die laufende Datei abbrechen.
    if (recording_) return false;
    return sendControlAction(RunCam::Action::CHANGE_MODE);
}
