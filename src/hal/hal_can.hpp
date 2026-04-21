#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>   // Teensy 4.x CAN Library

namespace HAL {

bool canInit(uint32_t baud = 500000);
bool canSend(uint32_t id, uint8_t* data, uint8_t len);
bool canReceive(uint32_t &id, uint8_t* data, uint8_t &len);

}