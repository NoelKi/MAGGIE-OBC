#include "system.hpp"

System::System() {
    // Konstruktor
}

System::~System() {
    // Destruktor
}

bool System::init() {
    startup_time = millis();
    
    printWelcomeBanner();
    
    // Initialisiere Logger zuerst
    if (!logger.init("MAGGIE_001")) {
        Serial.println("WARNING: Logging initialization failed");
    }
    
    // Initialisiere Mission Control
    if (!mission_control.init()) {
        Serial.println("FATAL: Mission Control initialization failed!");
        return false;
    }
    
    system_healthy = true;
    logger.logMessage("INFO", "System initialization complete");
    
    return true;
}

void System::run() {
    if (!system_healthy) return;
    
    // Hauptkontroll-Loop
    mission_control.run();
}

MissionControl* System::getMissionControl() {
    return &mission_control;
}

LoggingService* System::getLoggingService() {
    return &logger;
}

uint32_t System::getUptimeMs() const {
    return millis() - startup_time;
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