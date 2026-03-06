#include "hal_uart.h"
#include "hal_config.h"

namespace HAL {

bool uartInit(uint32_t baudrate) {
    try {
        // Starte Serial mit angegeben Baudrate
        Serial.begin(baudrate);
        
        // Warte kurz, bis Serial ready ist
        delay(100);
        
        // Welcome Message
        Serial.println("\n========================================");
        Serial.println("MAGGIE On-Board Computer - REXUS Rocket");
        Serial.println("========================================");
        Serial.println("[HAL_UART] Serial initialized at " + String(baudrate) + " baud");
        Serial.println("========================================\n");
        
        return true;
    } catch (...) {
        return false;
    }
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

void uartDebugPrint(const String& message) {
    Serial.println("[DEBUG] " + message);
}

} // namespace HAL
