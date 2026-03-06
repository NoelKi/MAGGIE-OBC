#include "hal_can.h"

FlexCAN CANbus(500000);  // default, Baud wird später konfiguriert
CAN_message_t msg;

namespace HAL {

bool canInit(uint32_t baud) {
    try {
        CANbus.begin();
        Serial.println("[HAL_CAN] CAN Bus initialized at " + String(baud) + " baud");
        return true;
    } catch (...) {
        Serial.println("[HAL_CAN] ERROR: Failed to initialize CAN bus");
        return false;
    }
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