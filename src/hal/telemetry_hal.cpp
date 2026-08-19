#include "hal/telemetry_hal.hpp"
#include <cstring>

TelemetryDownlink::TelemetryDownlink(HardwareSerial& serial)
    : serial_(serial) {
}

bool TelemetryDownlink::init(uint32_t baudrate) {
    // On the Teensy 4.1 Serial8 already maps to pins 34 (RX) / 35 (TX),
    // so no explicit pin assignment is needed.
    //
    // RXINV dreht NUR die Empfangsleitung in Hardware um (LPUART-Register),
    // die Sendeleitung bleibt unveraendert - genau das, was hier gebraucht
    // wird. Begruendung siehe UPLINK_RX_INVERTED in telemetry_hal.hpp.
    serial_.begin(baudrate, UPLINK_RX_INVERTED ? SERIAL_8N1_RXINV : SERIAL_8N1);
    initialized_ = true;
    return true;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Write a signed 16-bit value big-endian into buf[0..1].
static inline void put_be16(uint8_t* buf, int16_t value) {
    buf[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[1] = static_cast<uint8_t>(value & 0xFF);
}

// Write a signed 32-bit value big-endian into buf[0..3].
static inline void put_be32(uint8_t* buf, int32_t value) {
    buf[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    buf[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buf[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[3] = static_cast<uint8_t>(value & 0xFF);
}

uint8_t TelemetryDownlink::crc8(const uint8_t* data, size_t len) {
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

// ---------------------------------------------------------------------------
// Frame assembly
// ---------------------------------------------------------------------------

void TelemetryDownlink::sendFrame(uint8_t msgid1, uint8_t msgid2, uint8_t ack,
                                  const uint8_t data[DOWNLINK_DATA_SIZE],
                                  uint8_t status1, uint8_t status2) {
    if (!initialized_) return;

    uint8_t frame[DOWNLINK_FRAME_SIZE];

    frame[0] = DL_START;
    frame[1] = msgid1;
    frame[2] = msgid2;
    frame[3] = ack;

    // COUNTER (big-endian)
    frame[4] = static_cast<uint8_t>((counter_ >> 8) & 0xFF);
    frame[5] = static_cast<uint8_t>(counter_ & 0xFF);

    // TIME (lower 16 bit of millis(), big-endian)
    const uint16_t t = static_cast<uint16_t>(millis() & 0xFFFF);
    frame[6] = static_cast<uint8_t>((t >> 8) & 0xFF);
    frame[7] = static_cast<uint8_t>(t & 0xFF);

    // DATA (bytes 8..15)
    memcpy(&frame[8], data, DOWNLINK_DATA_SIZE);

    frame[16] = status1;
    frame[17] = status2;

    // CRC over MSGID1..STATUS2 (bytes 1..17 -> 17 bytes)
    frame[18] = crc8(&frame[1], 17);
    frame[19] = DL_END;

    serial_.write(frame, DOWNLINK_FRAME_SIZE);
    counter_++;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void TelemetryDownlink::sendImu(const IMUReading& reading,
                                uint8_t status1, uint8_t status2) {
    uint8_t data[DOWNLINK_DATA_SIZE];

    // ACCEL frame
    memset(data, 0, sizeof(data));
    put_be16(&data[0], reading.raw_accel_x);
    put_be16(&data[2], reading.raw_accel_y);
    put_be16(&data[4], reading.raw_accel_z);
    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::IMU),
              static_cast<uint8_t>(DownlinkImuMsg::ACCEL),
              0, data, status1, status2);

    // GYRO frame
    memset(data, 0, sizeof(data));
    put_be16(&data[0], reading.raw_gyro_x);
    put_be16(&data[2], reading.raw_gyro_y);
    put_be16(&data[4], reading.raw_gyro_z);
    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::IMU),
              static_cast<uint8_t>(DownlinkImuMsg::GYRO),
              0, data, status1, status2);
}

void TelemetryDownlink::sendMotor(int32_t position, int16_t speed, uint8_t state,
                                  uint8_t status1, uint8_t status2) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [pos(int32 BE) speed(int16 BE) state(uint8) spare]
    put_be32(&data[0], position);
    put_be16(&data[4], speed);
    data[6] = state;
    // data[7] reserved (0)

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::MOTOR),
              static_cast<uint8_t>(DownlinkMotorMsg::STATE),
              0, data, status1, status2);
}

void TelemetryDownlink::sendUplinkRaw(const uint8_t* bytes, uint8_t len) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    if (len > DOWNLINK_DATA_SIZE) len = DOWNLINK_DATA_SIZE;
    for (uint8_t i = 0; i < len; i++) data[i] = bytes[i];

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::SYS),
              static_cast<uint8_t>(DownlinkSysMsg::UPLINK_RAW),
              0, data, 0, len);   // STATUS2 traegt die gueltige Laenge
}

void TelemetryDownlink::sendUplinkStats(uint32_t rx_bytes, uint16_t frames_ok,
                                       uint16_t frames_bad, uint8_t last_opcode) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [rx_bytes(uint16 BE, gesaettigt) frames_ok(uint16 BE)
    //        frames_bad(uint16 BE) last_opcode(uint8) 0]
    const uint16_t rx16 = (rx_bytes > 0xFFFF) ? 0xFFFF : static_cast<uint16_t>(rx_bytes);
    data[0] = static_cast<uint8_t>(rx16 >> 8);
    data[1] = static_cast<uint8_t>(rx16 & 0xFF);
    data[2] = static_cast<uint8_t>(frames_ok >> 8);
    data[3] = static_cast<uint8_t>(frames_ok & 0xFF);
    data[4] = static_cast<uint8_t>(frames_bad >> 8);
    data[5] = static_cast<uint8_t>(frames_bad & 0xFF);
    data[6] = last_opcode;
    // data[7] reserved (0)

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::SYS),
              static_cast<uint8_t>(DownlinkSysMsg::UPLINK),
              0, data, 0, 0);
}

void TelemetryDownlink::sendSystem(uint8_t mission_state, uint8_t subsystems,
                                   uint32_t uptime_ms,
                                   uint8_t status1, uint8_t status2,
                                   uint8_t rexus) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [state(uint8) subsys(uint8) uptime_ms(uint32 BE) rexus(uint8) 0]
    data[0] = mission_state;
    data[1] = subsystems;
    put_be32(&data[2], static_cast<int32_t>(uptime_ms));
    data[6] = rexus;      // rohe REXUS-Pegel, siehe DL_REXUS_*
    // data[7] reserved (0)

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::SYS),
              static_cast<uint8_t>(DownlinkSysMsg::STATE),
              0, data, status1, status2);
}
