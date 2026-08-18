#pragma once

#include <cstdint>
#include <Arduino.h>
#include "hal/imu_hal.hpp"

/**
 * @file telemetry_hal.hpp
 * @brief MAGGIE Downlink Telemetry (UART)
 *
 * Sends telemetry over a hardware UART using the fixed 20-byte MAGGIE
 * downlink frame. On the Teensy 4.1 the downlink is wired to pins
 * 34 (RX) / 35 (TX) -> Serial8.
 *
 * ---------------------------------------------------------------------------
 * Downlink frame (20 bytes, fixed size)
 * ---------------------------------------------------------------------------
 *  Idx | Field    | Bytes | Description
 *  ----+----------+-------+------------------------------------------------
 *   0  | START    |  1    | Frame start marker (DL_START)
 *   1  | MSGID1   |  1    | Subsystem / message category (DownlinkSubsystem)
 *   2  | MSGID2   |  1    | Message type within subsystem
 *   3  | ACK      |  1    | Acknowledge flag (0 for pure telemetry)
 *  4-5 | COUNTER  |  2    | Frame counter, big-endian, increments per frame
 *  6-7 | TIME     |  2    | On-board time, big-endian (lower 16 bit of millis())
 * 8-15 | DATA     |  8    | Payload, layout depends on MSGID (big-endian)
 *  16  | STATUS1  |  1    | Status bitfield 1
 *  17  | STATUS2  |  1    | Status bitfield 2
 *  18  | CRC      |  1    | CRC-8 (poly 0x07, init 0x00) over bytes 1..17
 *  19  | END      |  1    | Frame end marker (DL_END)
 *
 * All multi-byte fields are transmitted big-endian (most significant byte first).
 *
 * DATA layout per message type:
 *   IMU/ACCEL:   [ax_hi ax_lo ay_hi ay_lo az_hi az_lo  0 0]  (int16 sensor counts)
 *   IMU/GYRO :   [gx_hi gx_lo gy_hi gy_lo gz_hi gz_lo  0 0]  (int16 sensor counts)
 *   MOTOR/STATE: [pos(int32 BE) speed(int16 BE) state(uint8) 0]  (Encoder-Counts, PWM, Bits)
 *
 * The int16 values are the native BMI088 counts. The ground station applies
 * the documented scale factors:
 *   accel [m/s^2] = count * (9.80665 / 10920)   (+/-3 g range)
 *   gyro  [deg/s] = count * (1 / 65.536)         (+/-500 deg/s range)
 */

// Frame geometry
static constexpr uint8_t DOWNLINK_FRAME_SIZE = 20;
static constexpr uint8_t DOWNLINK_DATA_SIZE  = 8;

// Frame markers
static constexpr uint8_t DL_START = 0x7E;
static constexpr uint8_t DL_END   = 0x7F;

// MSGID1 - subsystem / message category
enum class DownlinkSubsystem : uint8_t {
    IMU   = 0x01,
    MOTOR = 0x02,
};

// MSGID2 - message type for the IMU subsystem
enum class DownlinkImuMsg : uint8_t {
    ACCEL = 0x01,
    GYRO  = 0x02,
};

// MSGID2 - message type for the MOTOR subsystem
enum class DownlinkMotorMsg : uint8_t {
    STATE = 0x01,
};

// STATUS1 bit definitions
static constexpr uint8_t DL_STATUS1_SYSTEM_HEALTHY = 0x01;  ///< bit0: system healthy
static constexpr uint8_t DL_STATUS1_IMU_VALID      = 0x02;  ///< bit1: IMU reading valid

// MOTOR/STATE - state byte bit definitions (DATA[6])
static constexpr uint8_t DL_MOTOR_STATE_ON        = 0x01;  ///< bit0: motor dauerhaft an (on())
static constexpr uint8_t DL_MOTOR_STATE_MOVING    = 0x02;  ///< bit1: Closed-Loop-Fahrt aktiv
static constexpr uint8_t DL_MOTOR_STATE_AT_TARGET = 0x04;  ///< bit2: keine Fahrt aktiv / am Ziel

class TelemetryDownlink {
public:
    /**
     * @brief Constructor
     * @param serial Hardware serial port used for the downlink (e.g. Serial8)
     */
    explicit TelemetryDownlink(HardwareSerial& serial);

    /**
     * @brief Initialize the downlink UART
     * @param baudrate Serial speed (default 38400)
     * @return true if successful
     */
    bool init(uint32_t baudrate = 38400);

    /**
     * @brief Send the IMU telemetry for one reading.
     *
     * Emits two frames: one ACCEL frame and one GYRO frame.
     *
     * @param reading Latest IMU reading
     * @param status1 STATUS1 byte (see DL_STATUS1_* flags)
     * @param status2 STATUS2 byte
     */
    void sendImu(const IMUReading& reading, uint8_t status1 = 0, uint8_t status2 = 0);

    /**
     * @brief Send the current motor state as one MOTOR/STATE frame.
     *
     * @param position Encoder position in quadrature counts
     * @param speed    Current signed PWM speed (-255..255)
     * @param state    State bitfield (see DL_MOTOR_STATE_* flags)
     * @param status1  STATUS1 byte (see DL_STATUS1_* flags)
     * @param status2  STATUS2 byte
     */
    void sendMotor(int32_t position, int16_t speed, uint8_t state,
                   uint8_t status1 = 0, uint8_t status2 = 0);

private:
    HardwareSerial& serial_;
    uint16_t counter_ = 0;
    bool initialized_ = false;

    /**
     * @brief Assemble and transmit one 20-byte downlink frame.
     * @param data 8-byte payload (DOWNLINK_DATA_SIZE)
     */
    void sendFrame(uint8_t msgid1, uint8_t msgid2, uint8_t ack,
                   const uint8_t data[DOWNLINK_DATA_SIZE],
                   uint8_t status1, uint8_t status2);

    /// CRC-8, polynomial 0x07, init 0x00 (SMBus/CCITT style)
    static uint8_t crc8(const uint8_t* data, size_t len);
};
