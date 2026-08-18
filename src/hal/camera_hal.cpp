#include "hal/camera_hal.hpp"

CameraHAL::CameraHAL(const CameraConfig& config, CameraBus* bus)
    : camera_id_(config.camera_id),
      channel_(config.mux_channel),
      bus_(bus),
      model_(config.model),
      mode_(config.mode),
      boot_delay_ms_(config.boot_delay_ms),
      enabled_(config.enabled) {
}

bool CameraHAL::init() {
    if (!enabled_ || !bus_ || !bus_->isReady() || channel_ >= CameraBus::MAX_CHANNELS) {
        state_ = CameraState::UNCONFIGURED;
        return false;
    }

    // Die Kamera braucht nach dem Einschalten mehrere Sekunden, bis ihr UART
    // antwortet. Vorher gesendete Kommandos gehen verloren, deshalb erst nach
    // boot_delay_ms_ mit dem Handshake beginnen. Alle Kameras booten
    // gleichzeitig, die Wartezeit läuft also nur einmal für alle.
    init_ms_ = millis();
    state_ = CameraState::BOOTING;
    return true;
}

void CameraHAL::update(uint32_t now_ms) {
    if (state_ == CameraState::UNCONFIGURED) return;

    if (state_ == CameraState::BOOTING) {
        if ((now_ms - init_ms_) < boot_delay_ms_) return;
        state_ = CameraState::DETECTING;
        // Ersten Versuch sofort zulassen (Unterlauf ist hier gewollt).
        last_probe_ms_ = now_ms - PROBE_INTERVAL_MS;
    }

    if (state_ == CameraState::DETECTING) {
        if ((now_ms - last_probe_ms_) < PROBE_INTERVAL_MS) return;
        last_probe_ms_ = now_ms;
        ++probe_attempts_;

        if (performHandshake()) {
            device_detected_ = true;
            state_ = CameraState::READY;
        } else if (probe_attempts_ >= MAX_PROBE_ATTEMPTS) {
            // Keine Antwort. Das kann an einer defekten RX-Leitung oder am Mux
            // liegen, während TX zur Kamera funktioniert. Für einen einmaligen
            // Flug ist es besser, die Kommandos trotzdem blind zu senden, als
            // die Aufnahme zu verlieren. device_detected_ bleibt false und geht
            // so in die Telemetrie.
            state_ = CameraState::READY;
        } else {
            return;
        }
    }

    // Ab hier: state_ == READY
    if (mode_ == CameraTriggerMode::RECORD_ON_BOOT && !auto_record_done_) {
        // Erst als erledigt markieren, wenn das Kommando wirklich rausging -
        // sonst geht die Aufnahme verloren, falls das Rate-Limit gerade greift.
        auto_record_done_ = startRecording();
    }
}

bool CameraHAL::probe() {
    if (state_ == CameraState::UNCONFIGURED) return false;
    device_detected_ = performHandshake();
    return device_detected_;
}

bool CameraHAL::performHandshake() {
    if (!bus_ || !bus_->select(channel_)) return false;

    HardwareSerial* serial = bus_->serial();
    if (!serial) return false;

    bus_->flushInput();
    if (!sendCommand(RunCam::Cmd::GET_DEVICE_INFO, nullptr, 0)) return false;

    // Blockierend lesen: der Mux-Kanal liegt nur JETZT an. Würde die Antwort
    // über mehrere update()-Durchläufe eingesammelt, könnte zwischendurch eine
    // andere Kamera den Bus umschalten und die Antwort ginge verloren.
    uint8_t buffer[RunCam::DEVICE_INFO_RESPONSE_LENGTH] = {};
    size_t count = 0;
    const uint32_t deadline = millis() + RESPONSE_TIMEOUT_MS;

    while (count < RunCam::DEVICE_INFO_RESPONSE_LENGTH &&
           static_cast<int32_t>(millis() - deadline) < 0) {
        if (serial->available() <= 0) continue;

        const int c = serial->read();
        if (c < 0) continue;

        // Auf den Header synchronisieren: alles vor 0xCC ist Müll.
        if (count == 0 && static_cast<uint8_t>(c) != RunCam::HEADER) continue;

        buffer[count++] = static_cast<uint8_t>(c);
    }

    if (count < RunCam::DEVICE_INFO_RESPONSE_LENGTH) return false;

    // CRC läuft über alle Bytes inkl. Header, das letzte Byte ist der CRC selbst.
    const uint8_t crc = RunCam::crc8DvbS2Buffer(buffer,
                                                RunCam::DEVICE_INFO_RESPONSE_LENGTH - 1);
    if (crc != buffer[RunCam::DEVICE_INFO_RESPONSE_LENGTH - 1]) return false;

    protocol_version_ = buffer[1];
    features_ = static_cast<uint16_t>(buffer[2]) |
                (static_cast<uint16_t>(buffer[3]) << 8);
    return true;
}

bool CameraHAL::sendCommand(uint8_t command, const uint8_t* payload, size_t payload_length) {
    if (state_ == CameraState::UNCONFIGURED || !bus_) return false;
    if (payload_length + 3 > RunCam::MAX_TX_PACKET_LENGTH) return false;

    // Ohne den richtigen Kanal ginge das Kommando an die falsche Kamera.
    if (!bus_->select(channel_)) return false;

    HardwareSerial* serial = bus_->serial();
    if (!serial) return false;

    uint8_t packet[RunCam::MAX_TX_PACKET_LENGTH];
    size_t length = 0;

    packet[length++] = RunCam::HEADER;
    packet[length++] = command;
    for (size_t i = 0; i < payload_length; ++i) {
        packet[length++] = payload[i];
    }
    packet[length] = RunCam::crc8DvbS2Buffer(packet, length);
    ++length;

    if (serial->write(packet, length) != length) return false;

    // Rausschreiben, bevor irgendjemand den Mux weiterschaltet.
    serial->flush();
    return true;
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
