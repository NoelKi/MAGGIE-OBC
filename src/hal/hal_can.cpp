#include "hal_can.hpp"

// FlexCAN_T4 für Teensy 4.x — nutzt CAN1 (Pins 22/23)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANbus;

namespace HAL {

bool canInit(uint32_t baud) {
    CANbus.begin();
    CANbus.setBaudRate(baud);
    Serial.println("[HAL_CAN] CAN Bus initialized (FlexCAN_T4)");
    return true;
}

bool canSend(uint32_t id, uint8_t* data, uint8_t len) {
    CAN_message_t tx;
    tx.id = id;
    tx.len = len;
    memcpy(tx.buf, data, len);
    return (CANbus.write(tx) > 0);
}

bool canReceive(uint32_t &id, uint8_t* data, uint8_t &len) {
    CAN_message_t rx;
    if (CANbus.read(rx)) {
        id = rx.id;
        len = rx.len;
        memcpy(data, rx.buf, len);
        return true;
    }
    return false;
}

}