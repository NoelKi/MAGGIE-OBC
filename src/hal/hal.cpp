#include "hal.h"

namespace HAL {

bool initializeAll() {
    Serial.begin(115200);
    delay(100);
    
    Serial.println("\n========================================");
    Serial.println("MAGGIE OBC - Hardware Abstraction Layer");
    Serial.println("========================================\n");
    
    bool allSuccess = true;
    
    // Starte GPIO
    Serial.print("[INIT] GPIO... ");
    // GPIO hat keine Init, aber LED Setup
    Serial.println("OK");
    
    // Starte UART
    Serial.print("[INIT] UART... ");
    if (!uartInit(HAL_UART_BAUD)) {
        Serial.println("FAILED");
        allSuccess = false;
    } else {
        Serial.println("OK");
    }
    
    // Starte I2C
    Serial.print("[INIT] I2C... ");
    if (!i2cInit()) {
        Serial.println("FAILED");
        allSuccess = false;
    } else {
        Serial.println("OK");
    }
    
    // Starte SPI
    Serial.print("[INIT] SPI... ");
    if (!spiInit()) {
        Serial.println("FAILED");
        allSuccess = false;
    } else {
        Serial.println("OK");
    }
    
    // Starte ADC
    Serial.print("[INIT] ADC... ");
    if (!adcInit(ADC_RES_12BIT)) {
        Serial.println("FAILED");
        allSuccess = false;
    } else {
        Serial.println("OK");
    }
    
    // Starte PWM
    Serial.print("[INIT] PWM... ");
    if (!pwmInit()) {
        Serial.println("FAILED");
        allSuccess = false;
    } else {
        Serial.println("OK");
    }
    
    // Starte CAN
    Serial.print("[INIT] CAN... ");
    if (!canInit(HAL_CAN_BAUD)) {
        Serial.println("FAILED");
        allSuccess = false;
    } else {
        Serial.println("OK");
    }
    
    Serial.println("\n========================================");
    if (allSuccess) {
        Serial.println("✓ All HAL modules initialized successfully");
    } else {
        Serial.println("✗ Some HAL modules failed to initialize");
    }
    Serial.println("========================================\n");
    
    return allSuccess;
}

} // namespace HAL
