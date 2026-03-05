#include "hal_can.h"

FlexCAN CANbus(500000);  // default, Baud wird später konfiguriert
CAN_message_t msg;

namespace HAL {

void canInit(uint32_t baud) {
    CANbus.begin();
}

bool canSend(uint32_t id, uint8_t* data, uint8_t len) {
    CAN_message_t tx;
    tx.id = id;
    tx.len = len;
    memcpy(tx.buf, data, len);
    return CANbus.write(tx);
}

bool canReceive(uint32_t &id, uint8_t* data, uint8_t &len) {
    if (CANbus.read(msg)) {
        id = msg.id;
        len = msg.len;
        memcpy(data, msg.buf, len);
        return true;
    }
    return false;
}

}