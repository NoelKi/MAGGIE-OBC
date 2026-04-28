#include "system.hpp"
#include <core_pins.h>
#include <usb_seremu.h>
#include <usb_serial.h>

System::System() {
    // Konstruktor
}

System::~System() {
    // Destruktor
}

bool System::init() {
    startup_time = millis();
    
    printWelcomeBanner();
    
    
    system_healthy = true;
    
    return true;
}

void System::run() {
    if (!system_healthy) return;
}

void System::printWelcomeBanner() {
    Serial.println("\n");
    Serial.println("╔═════════════════════════════════════════════════╗");
    Serial.println("║             MAGGIE On-Board Computer            ║");
    Serial.println("║     REXUS Program - Rocket Experiment System    ║");
    Serial.println("║                     v 1.0                       ║");
    Serial.println("╚═════════════════════════════════════════════════╝");
    Serial.println("");
    Serial.println("Built by MAGGIE Team on Teensy 4.1");
    Serial.println("Copyright 2026 - All Rights Reserved");
    Serial.println("");
}