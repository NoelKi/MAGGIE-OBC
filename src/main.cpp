#include <Arduino.h>
#include "system.h"

// Globale System-Instanz
System* g_system = nullptr;

void setup() {
    // Initialisiere Serial für Debug
    Serial.begin(115200);
    delay(500);  // Warte auf Serial-Monitor
    
    // Initialisiere System
    g_system = new System();
    if (!g_system->init()) {
        Serial.println("FATAL: System initialization failed!");
        while (true) {
            delay(1000);
            Serial.println("System halted");
        }
    }
}

void loop() {
    // if (g_system) {
    //     g_system->run();
    // } else {
    //     delay(100);
    // }

    Serial.printf("Der Loop ist am Loopen Penis\n");
}

