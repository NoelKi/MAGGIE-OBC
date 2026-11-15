#include "hal/telemetry_hal.hpp"
#include <cstring>

TelemetryDownlink::TelemetryDownlink(HardwareSerial& serial)
    : serial_(serial) {
}

bool TelemetryDownlink::init(uint32_t baudrate) {
    // On the Teensy 4.1 Serial4 already maps to pins 16 (RX) / 17 (TX),
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
                                  uint8_t status1, uint8_t status2, bool sd_only) {
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

    // SD-Karte bekommt jedes Frame.
    if (tap_) tap_(tap_ctx_, frame, DOWNLINK_FRAME_SIZE);
    if (sd_only) return;   // Ereignis-Frames: kein UART, kein Zaehler

    // UART: nie blockieren (write() wuerde warten, bis Platz ist). Zwei Spuren:
    //  - SYS-Frames (Zustand, REXUS-Signale) und ARM/JOINT: immer, sobald das
    //    Frame passt. ARM/JOINT, weil die Gelenkwinkel sonst in der vollen
    //    Leitung untergehen (Test 15.11.2026: kein einziges Frame in 5 min) -
    //    hoechstens ARM_MAX_JOINTS x 5 Hz und nur bei neuem Wert, real ~200 B/s.
    //  - Alles andere (IMU, Kraft, Motor): nur solange die Warteschlange kurz
    //    ist (DOWNLINK_NORMAL_QUEUE_MAX). Sonst stuende ein SYS-Frame hinter
    //    hunderten Byte Sensordaten an - und waere bei voller Leitung so oft
    //    weg wie jedes andere Frame. Verworfenes fehlt nur im Downlink (der
    //    Boden sieht eine Zaehler-Luecke), die SD-Karte hat es.
    const int free_bytes = serial_.availableForWrite();
    if (free_bytes > tx_capacity_) tx_capacity_ = free_bytes;   // leerer Puffer = Kapazitaet
    const int queued = tx_capacity_ - free_bytes;
    const bool priority =
        msgid1 == static_cast<uint8_t>(DownlinkSubsystem::SYS) ||
        (msgid1 == static_cast<uint8_t>(DownlinkSubsystem::ARM) &&
         msgid2 == static_cast<uint8_t>(DownlinkArmMsg::JOINT));

    const bool send = priority
        ? free_bytes >= DOWNLINK_FRAME_SIZE
        : queued + DOWNLINK_FRAME_SIZE <= DOWNLINK_NORMAL_QUEUE_MAX;
    if (send) {
        serial_.write(frame, DOWNLINK_FRAME_SIZE);
    } else {
        uart_dropped_++;
    }
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

void TelemetryDownlink::sendForce(DownlinkForceMsg msg, const ForceReading& reading,
                                  bool stale, bool tared, uint8_t status1) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: bis zu 4x int16 BE in Kanalreihenfolge. Nicht belegte Kanaele
    // bleiben 0 - TARGET1 nutzt drei, TARGET2 alle vier.
    uint8_t flags = 0;
    const uint8_t n = reading.count < FORCE_MAX_CHANNELS
                    ? reading.count : FORCE_MAX_CHANNELS;
    for (uint8_t i = 0; i < n; i++) {
        put_be16(&data[i * 2], reading.tele[i]);
        if (reading.sat[i]) flags |= static_cast<uint8_t>(DL_FORCE_SAT_0 << i);
    }

    if (tared) flags |= DL_FORCE_TARED;
    if (stale) flags |= DL_FORCE_STALE;

    // Flags als STATUS2 - DATA ist bei TARGET2 vollstaendig belegt.
    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::FORCE),
              static_cast<uint8_t>(msg),
              0, data, status1, flags);
}

void TelemetryDownlink::sendMotor(DownlinkMotorMsg msg, int32_t position, int16_t speed,
                                  uint8_t state, uint8_t status1, uint8_t status2) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [pos(int32 BE) speed(int16 BE) state(uint8) spare]
    put_be32(&data[0], position);
    put_be16(&data[4], speed);
    data[6] = state;
    // data[7] reserved (0)

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::MOTOR),
              static_cast<uint8_t>(msg),
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

// ---------------------------------------------------------------------------
// Roboterarm - Werte unveraendert von der RCU, nur auf big-endian gedreht
// ---------------------------------------------------------------------------

static void putU16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}

static void putU32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v & 0xFF);
}

void TelemetryDownlink::sendArmJoint(uint8_t node, const ArmJoint& joint) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [node state angle_urad(int32 BE) 0 0]
    data[0] = node;
    data[1] = joint.state;
    putU32(&data[2], static_cast<uint32_t>(joint.angle_urad));

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::ARM),
              static_cast<uint8_t>(DownlinkArmMsg::JOINT),
              0, data, 0, 0);
}

void TelemetryDownlink::sendArmVoltage(uint8_t node, const ArmJoint& joint) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [node 0 bus_mv(uint16 BE) stepper_mv(uint16 BE) 0 0]
    data[0] = node;
    putU16(&data[2], joint.bus_mv);
    putU16(&data[4], joint.stepper_mv);

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::ARM),
              static_cast<uint8_t>(DownlinkArmMsg::VOLTAGE),
              0, data, 0, 0);
}

void TelemetryDownlink::sendArmTemp(uint8_t node, const ArmJoint& joint) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [node 0 mcu(int16 BE) imu(int16 BE) 0 0]
    data[0] = node;
    putU16(&data[2], static_cast<uint16_t>(joint.mcu_temp));
    putU16(&data[4], static_cast<uint16_t>(joint.imu_temp));

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::ARM),
              static_cast<uint8_t>(DownlinkArmMsg::TEMP),
              0, data, 0, 0);
}

void TelemetryDownlink::sendArmRcuImu(const ArmRcuImu& imu) {
    uint8_t data[DOWNLINK_DATA_SIZE];

    // DATA: [x y z (int16 BE) 0 0] - je ein Frame fuer Accel und Gyro
    memset(data, 0, sizeof(data));
    for (uint8_t i = 0; i < 3; i++) putU16(&data[2 * i], static_cast<uint16_t>(imu.acc[i]));
    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::ARM),
              static_cast<uint8_t>(DownlinkArmMsg::RCU_ACCEL),
              0, data, 0, 0);

    memset(data, 0, sizeof(data));
    for (uint8_t i = 0; i < 3; i++) putU16(&data[2 * i], static_cast<uint16_t>(imu.gyro[i]));
    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::ARM),
              static_cast<uint8_t>(DownlinkArmMsg::RCU_GYRO),
              0, data, 0, 0);
}

void TelemetryDownlink::sendArmLink(const ArmLink& link, uint32_t now_ms) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [frames_ok(uint16 BE) frames_bad(uint16 BE) last_msg last_node
    //        silence(uint8, 0,1 s, gesaettigt) freefall_count]
    // silence = Zeit seit dem letzten gueltigen Frame; 255 heisst ">= 25,5 s"
    // oder "noch nie eins empfangen" (dann steht frames_ok auf 0).
    putU16(&data[0], link.framesOk());
    putU16(&data[2], link.framesBad());
    data[4] = link.lastMsg();
    data[5] = link.lastNode();
    const uint32_t silence = link.framesOk() ? (now_ms - link.lastFrameMs()) / 100 : 0xFF;
    data[6] = silence > 0xFF ? 0xFF : static_cast<uint8_t>(silence);
    data[7] = link.freefallCount();

    const uint8_t status2 = link.linkAlive(now_ms) ? DL_ARM_LINK_ALIVE : 0;
    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::SYS),
              static_cast<uint8_t>(DownlinkSysMsg::ARM_LINK),
              0, data, 0, status2);
}

void TelemetryDownlink::sendArmTraj(uint8_t phase, uint8_t result, uint8_t node, uint16_t step,
                                    float max_err_deg, uint8_t runs) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    // DATA: [phase result node step(uint16 BE) max_err(uint16 BE, 0,01 Grad) runs]
    data[0] = phase;
    data[1] = result;
    data[2] = node;
    putU16(&data[3], step);
    float cdeg = max_err_deg * 100.0f + 0.5f;
    if (cdeg < 0.0f) cdeg = 0.0f;
    if (cdeg > 65535.0f) cdeg = 65535.0f;
    putU16(&data[5], static_cast<uint16_t>(cdeg));
    data[7] = runs;

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::SYS),
              static_cast<uint8_t>(DownlinkSysMsg::ARM_TRAJ),
              0, data, 0, 0);
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

void TelemetryDownlink::logEvent(uint8_t state, uint8_t prev_state, uint8_t signals,
                                 uint8_t prev_signals, uint32_t millis_now) {
    uint8_t data[DOWNLINK_DATA_SIZE];
    memset(data, 0, sizeof(data));

    data[0] = state;
    data[1] = prev_state;
    data[2] = signals;
    data[3] = prev_signals;
    put_be32(&data[4], static_cast<int32_t>(millis_now));

    sendFrame(static_cast<uint8_t>(DownlinkSubsystem::EVENT),
              static_cast<uint8_t>(DownlinkEventMsg::STATE),
              0, data, 0, 0, /*sd_only=*/true);
}
