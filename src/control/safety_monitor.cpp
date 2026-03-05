#include "control/safety_monitor.h"

SafetyMonitor::SafetyMonitor() {
    // Konstruktor
}

SafetyMonitor::~SafetyMonitor() {
    // Destruktor
}

bool SafetyMonitor::init() {
    Serial.println("INFO: Safety monitor initialized");
    last_check_time = millis();
    return true;
}

void SafetyMonitor::update() {
    uint32_t now = millis();
    
    // Prüfe Watchdog
    if (now - last_check_time > watchdog_timeout) {
        Serial.println("WARNING: Watchdog timeout - possible system hang");
        onSafetyLevelChange(SafetyLevel::WARNING);
    }
    
    last_check_time = now;
    
    // TODO: Implementiere regelmäßige Checks
    // - Prüfe Sensorstatus
    // - Überwache Speicher
    // - Prüfe Stromversorgung
}

SafetyLevel SafetyMonitor::checkSensorBounds(float accel_magnitude, float temperature, float altitude) {
    SafetyLevel level = SafetyLevel::NOMINAL;
    
    // Prüfe Beschleunigung
    if (accel_magnitude > accel_threshold) {
        Serial.printf("WARNING: Acceleration threshold exceeded: %.2f m/s²\n", accel_magnitude);
        level = SafetyLevel::WARNING;
    }
    
    // Prüfe Temperatur
    if (temperature < temp_min || temperature > temp_max) {
        Serial.printf("WARNING: Temperature out of bounds: %.2f °C\n", temperature);
        level = SafetyLevel::CRITICAL;
    }
    
    // Prüfe Höhe
    if (altitude > altitude_max) {
        Serial.printf("WARNING: Altitude exceeded: %.2f m\n", altitude);
        level = SafetyLevel::CRITICAL;
    }
    
    if (level != SafetyLevel::NOMINAL) {
        onSafetyLevelChange(level);
    }
    
    return level;
}

bool SafetyMonitor::checkPowerSupply() {
    // TODO: Implementiere Spannungs-Überwachung
    // - Lese ADC-Pin für Batterie-Spannung
    // - Prüfe auf kritisch niedrige Spannungen
    
    return true;  // Placeholder
}

bool SafetyMonitor::checkMemory() {
    // TODO: Implementiere Speicher-Check
    // - Prüfe verfügbaren RAM
    // - Prüfe SD-Karten-Speicher
    
    return true;  // Placeholder
}

void SafetyMonitor::emergencyShutdown() {
    Serial.println("EMERGENCY: Emergency shutdown initiated!");
    onSafetyLevelChange(SafetyLevel::EMERGENCY);
    
    // TODO: Implementiere Notfall-Abschaltung
    // - Deaktiviere alle Geräte
    // - Stoppe Datenerfassung
    // - Schreibe Fehler-Log
    // - Aktiviere sichere Pins (z.B. Sicherungs-Pin)
    
    // Infinite Loop bis zu manueller Reset
    while (true) {
        delay(1000);
        Serial.println("EMERGENCY: System shutdown");
    }
}

SafetyLevel SafetyMonitor::getCurrentLevel() const {
    return current_level;
}

void SafetyMonitor::setAccelThreshold(float threshold) {
    accel_threshold = threshold;
}

void SafetyMonitor::setTemperatureThreshold(float min_temp, float max_temp) {
    temp_min = min_temp;
    temp_max = max_temp;
}

void SafetyMonitor::setAltitudeThreshold(float max_altitude) {
    altitude_max = max_altitude;
}

void SafetyMonitor::onSafetyLevelChange(SafetyLevel new_level) {
    if (current_level == new_level) return;
    
    current_level = new_level;
    
    Serial.print("INFO: Safety level changed to: ");
    switch (new_level) {
        case SafetyLevel::NOMINAL:
            Serial.println("NOMINAL");
            break;
        case SafetyLevel::WARNING:
            Serial.println("WARNING");
            break;
        case SafetyLevel::CRITICAL:
            Serial.println("CRITICAL");
            break;
        case SafetyLevel::EMERGENCY:
            Serial.println("EMERGENCY");
            break;
    }
}
