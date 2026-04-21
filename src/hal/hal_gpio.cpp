#include <Arduino.h>
#include "hal_gpio.hpp"

namespace HAL {

void setLed(bool on) {
    digitalWrite(13, on ? HIGH : LOW);
}

}