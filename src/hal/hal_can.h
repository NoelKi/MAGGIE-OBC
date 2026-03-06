#pragma once
#include <Arduino.h>
#include <FlexCAN.h>   // Teensy 4.1 FlexCAN Library

namespace HAL {

bool canInit(uint32_t baud);
bool canSend(uint32_t id, uint8_t* data, uint8_t len);
bool canReceive(uint32_t &id, uint8_t* data, uint8_t &len);

}