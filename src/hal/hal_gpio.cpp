#include <Arduino.h>
#include "hal_gpio.h"

namespace HAL {

void setLed(bool on) {
    digitalWrite(13, on ? HIGH : LOW);
}

}