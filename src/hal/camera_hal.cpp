#include "hal/camera_hal.hpp"

CameraHAL::CameraHAL(const CameraConfig& config)
    : camera_id_(config.camera_id),
      serial_(config.serial),
      baudrate_(config.baudrate),
      mode_(config.mode),
      interval_ms_(config.interval_ms) {
}

bool CameraHAL::init() {
    if (!serial_) {
        // Kein UART-Port konfiguriert (siehe camera_config.hpp) - Kamera bleibt inaktiv.
        initialized_ = false;
        return false;
    }
    serial_->begin(baudrate_);
    initialized_ = true;
    return true;
}

void CameraHAL::update(uint32_t now_ms) {
    if (!initialized_) return;

    switch (mode_) {
        case CameraTriggerMode::SNAPSHOT_INTERVAL:
            if (interval_ms_ > 0 && (now_ms - last_trigger_ms_) >= interval_ms_) {
                last_trigger_ms_ = now_ms;
                captureSnapshot();
            }
            break;

        case CameraTriggerMode::CONTINUOUS:
            if (!continuous_started_) {
                continuous_started_ = startRecording();
            }
            break;

        case CameraTriggerMode::MANUAL_ONLY:
        default:
            break;
    }
}

size_t CameraHAL::sendCommand(const uint8_t* command, size_t length) {
    if (!initialized_ || !serial_ || !command) return 0;
    return serial_->write(command, length);
}

size_t CameraHAL::receiveData(uint8_t* buffer, size_t max_length) {
    if (!initialized_ || !serial_ || !buffer) return 0;
    return serial_->readBytes(buffer, max_length);
}

bool CameraHAL::available() {
    if (!initialized_ || !serial_) return false;
    return serial_->available() > 0;
}

bool CameraHAL::startRecording() {
    return sendCommand(CamCmd::START_RECORDING, sizeof(CamCmd::START_RECORDING)) > 0;
}

bool CameraHAL::stopRecording() {
    continuous_started_ = false;
    return sendCommand(CamCmd::STOP_RECORDING, sizeof(CamCmd::STOP_RECORDING)) > 0;
}

bool CameraHAL::captureSnapshot() {
    return sendCommand(CamCmd::SNAPSHOT, sizeof(CamCmd::SNAPSHOT)) > 0;
}
