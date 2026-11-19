#include "hal/arm_hal.hpp"
#include <cstring>

static int16_t readI16(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

static uint16_t readU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

static int32_t readI32(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                                (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24));
}

static void writeU32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

ArmLink::ArmLink(HardwareSerial& serial)
    : serial_(serial) {
}

bool ArmLink::init(uint32_t baudrate) {
    serial_.begin(baudrate);
    initialized_ = true;
    return true;
}

uint32_t ArmLink::crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint32_t>(data[i]) << 24;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
        }
    }
    return crc;
}

int ArmLink::payloadSize(uint8_t msg_id) {
    // Nur was die RCU SENDET. Befehls-IDs kommen nie zurueck und gelten hier
    // als unbekannt - so faellt ein Echo oder Fehlverkabelung (TX auf RX) als
    // "unbekannt" auf, statt als gueltiger Befehl durchzugehen.
    switch (static_cast<ArmMsg>(msg_id)) {
        case ArmMsg::JOINT_STATE:          return 5;    // ICD v0.9: vorher 4
        case ArmMsg::IMU_DATA:             return 16;
        case ArmMsg::VOLTAGE_DATA:         return 4;
        case ArmMsg::TEMPERATURE_DATA:     return 4;
        case ArmMsg::REFERENCE_ANGLE_DATA: return 4;
        case ArmMsg::IDENTIFIED:           return 16;
        case ArmMsg::RCU_IMU_DATA:         return 12;
        case ArmMsg::RCU_FREEFALL_EVENT:   return 0;
        default:                           return -1;
    }
}

// ---------------------------------------------------------------------------
// Empfang
// ---------------------------------------------------------------------------

void ArmLink::poll() {
    if (!initialized_) return;

    while (serial_.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(serial_.read());

        if (len_ == 0 && b != ARM_SYNC) continue;
        buf_[len_++] = b;
        parseBuffer();
    }
}

void ArmLink::parseBuffer() {
    // Schleife statt einmal pruefen: Nach einem Resync kann der Rest im Puffer
    // schon ein vollstaendiges Frame sein. len_ bleibt dabei immer unter der
    // Laenge des aktuellen Kandidaten, buf_ laeuft also nie ueber.
    while (len_ >= ARM_HEADER_SIZE) {
        const int size = payloadSize(buf_[2]);
        if (size < 0) {
            frames_bad_++;
            drop(1);
            continue;
        }

        const uint8_t frame_len = static_cast<uint8_t>(ARM_HEADER_SIZE + size + ARM_CRC_SIZE);
        if (len_ < frame_len) return;

        const uint32_t expected = static_cast<uint32_t>(readI32(&buf_[ARM_HEADER_SIZE + size]));
        if (crc32(buf_, ARM_HEADER_SIZE + size) == expected) {
            handleFrame(buf_[1], buf_[2], &buf_[ARM_HEADER_SIZE]);
            drop(frame_len);
        } else {
            frames_bad_++;
            drop(1);
        }
    }
}

void ArmLink::drop(uint8_t n) {
    uint8_t i = n;
    while (i < len_ && buf_[i] != ARM_SYNC) i++;
    len_ = static_cast<uint8_t>(len_ - i);
    memmove(buf_, &buf_[i], len_);
}

void ArmLink::handleFrame(uint8_t node, uint8_t msg_id, const uint8_t* p) {
    const uint32_t now = millis();
    last_frame_ms_ = now;
    ever_received_ = true;
    if (frames_ok_ < 0xFFFF) frames_ok_++;
    last_msg_  = msg_id;
    last_node_ = node;

    ArmJoint* j = joint(node);

    switch (static_cast<ArmMsg>(msg_id)) {
        case ArmMsg::JOINT_STATE:
            if (!j) return;
            j->seen       = true;
            j->state      = p[0];
            j->angle_urad = readI32(&p[1]);
            j->state_ms   = now;
            j->state_new  = true;
            return;

        case ArmMsg::VOLTAGE_DATA:
            if (!j) return;
            j->bus_mv      = readU16(&p[0]);
            j->stepper_mv  = readU16(&p[2]);
            j->voltage_new = true;
            return;

        case ArmMsg::TEMPERATURE_DATA:
            if (!j) return;
            j->mcu_temp = readI16(&p[0]);
            j->imu_temp = readI16(&p[2]);
            j->temp_new = true;
            return;

        case ArmMsg::RCU_IMU_DATA:
            for (uint8_t i = 0; i < 3; i++) {
                rcu_imu_.acc[i]  = readI16(&p[2 * i]);
                rcu_imu_.gyro[i] = readI16(&p[6 + 2 * i]);
            }
            rcu_imu_.updated_ms = now;
            rcu_imu_.fresh = true;
            return;

        case ArmMsg::RCU_FREEFALL_EVENT:
            if (freefall_count_ < 0xFF) freefall_count_++;
            return;

        default:
            // IMU_DATA, REFERENCE_ANGLE_DATA, IDENTIFIED: gueltig und gezaehlt,
            // aber (noch) nicht ausgewertet.
            return;
    }
}

ArmJoint* ArmLink::joint(uint8_t node) {
    if (node < 1 || node > ARM_MAX_JOINTS) return nullptr;
    return &joints_[node - 1];
}

bool ArmLink::linkAlive(uint32_t now_ms) const {
    // Vorzeichenbehaftet: last_frame_ms_ kann juenger als now_ms sein, wenn
    // now_ms vor poll() genommen wurde (siehe ArmTrajPlayer::update).
    return ever_received_ &&
           static_cast<int32_t>(now_ms - last_frame_ms_) < static_cast<int32_t>(ARM_LINK_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// Senden
// ---------------------------------------------------------------------------

bool ArmLink::send(uint8_t node, ArmMsg msg, const uint8_t* payload, uint8_t len) {
    if (!initialized_ || len > ARM_MAX_PAYLOAD) return false;

    uint8_t frame[ARM_MAX_FRAME];
    frame[0] = ARM_SYNC;
    frame[1] = node;
    frame[2] = static_cast<uint8_t>(msg);
    frame[3] = 0x00;
    if (len) memcpy(&frame[ARM_HEADER_SIZE], payload, len);
    writeU32(&frame[ARM_HEADER_SIZE + len], crc32(frame, ARM_HEADER_SIZE + len));

    // Nie blockieren: passt das Frame nicht ganz in den Puffer, gar nicht
    // senden. Ein halbes Frame waere fuer die RCU nur ein CRC-Fehler.
    const uint8_t frame_len = static_cast<uint8_t>(ARM_HEADER_SIZE + len + ARM_CRC_SIZE);
    if (serial_.availableForWrite() < frame_len) {
        if (tx_dropped_ < 0xFFFF) tx_dropped_++;
        return false;
    }
    serial_.write(frame, frame_len);
    return true;
}

int ArmLink::txFree() const {
    return initialized_ ? serial_.availableForWrite() : 0;
}

bool ArmLink::sendCommand(uint8_t node, ArmMsg msg) {
    return send(node, msg, nullptr, 0);
}

bool ArmLink::sendMoveRel(uint8_t node, int32_t angle_urad) {
    uint8_t p[4];
    writeU32(p, static_cast<uint32_t>(angle_urad));
    return send(node, ArmMsg::STEPPER_MOVE_REL, p, sizeof(p));
}

bool ArmLink::sendMoveAbs(uint8_t node, int32_t angle_urad) {
    uint8_t p[4];
    writeU32(p, static_cast<uint32_t>(angle_urad));
    return send(node, ArmMsg::STEPPER_MOVE_ABS, p, sizeof(p));
}

bool ArmLink::sendSetVelocity(uint8_t node, float speed_deg_s, float accel_deg_s2) {
    // Auf der Leitung urad/s und urad/s^2 (Test 15.11.2026: ganze Grad/s
    // liess das Gelenk nur kriechen, urad-Werte fuhren es).
    static constexpr float URAD_PER_DEG = 17453.2925f;
    if (speed_deg_s < 0.0f) speed_deg_s = 0.0f;
    if (accel_deg_s2 < 0.0f) accel_deg_s2 = 0.0f;
    uint8_t p[8];
    writeU32(&p[0], static_cast<uint32_t>(speed_deg_s * URAD_PER_DEG + 0.5f));
    writeU32(&p[4], static_cast<uint32_t>(accel_deg_s2 * URAD_PER_DEG + 0.5f));
    return send(node, ArmMsg::STEPPER_SET_VELOCITY, p, sizeof(p));
}

void ArmLink::stopAll() {
    for (uint8_t node = 1; node <= ARM_MAX_JOINTS; node++) {
        sendCommand(node, ArmMsg::STOP);
    }
}

void ArmLink::inhibitAll() {
    for (uint8_t node = 1; node <= ARM_MAX_JOINTS; node++) {
        sendCommand(node, ArmMsg::INHIBIT);
    }
}
