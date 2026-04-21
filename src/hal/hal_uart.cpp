#include "hal_uart.hpp"
#include "hal_config.hpp"

namespace HAL {

bool uartInit(uint32_t baudrate) {
    Serial.begin(baudrate);
    delay(100);

    Serial.println("\n========================================");
    Serial.println("MAGGIE On-Board Computer - REXUS Rocket");
    Serial.println("========================================");

    char buf[64];
    snprintf(buf, sizeof(buf), "[HAL_UART] Serial initialized at %lu baud", (unsigned long)baudrate);
    Serial.println(buf);

    Serial.println("========================================\n");
    return true;
}

void uartWrite(char c) {
    Serial.write(c);
}

void uartWriteString(const char* str) {
    if (str) {
        Serial.print(str);
    }
}

int16_t uartRead() {
    if (Serial.available() > 0) {
        return Serial.read();
    }
    return -1;
}

uint16_t uartReadBytes(uint8_t* buffer, uint16_t len) {
    if (!buffer || len == 0) {
        return 0;
    }
    
    uint16_t bytesRead = 0;
    
    while (Serial.available() > 0 && bytesRead < len) {
        buffer[bytesRead] = Serial.read();
        bytesRead++;
    }
    
    return bytesRead;
}

uint16_t uartAvailable() {
    return Serial.available();
}

void uartWriteData(const uint8_t* data, uint16_t len) {
    if (!data || len == 0) {
        return;
    }
    
    for (uint16_t i = 0; i < len; i++) {
        Serial.write(data[i]);
    }
}

void uartFlush() {
    Serial.flush();
}

void uartDebugPrint(const char* message) {
    Serial.print("[DEBUG] ");
    Serial.println(message);
}

} // namespace HAL
