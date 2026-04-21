#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Sicherheits- und Watchdog-Modul
 * Überwacht System-Zustand und erzwingt Sicherheits-Maßnahmen
 */

enum class SafetyLevel {
    NOMINAL,        // Normalbetrieb
    WARNING,        // Warnung, aber noch sicher
    CRITICAL,       // Kritisch, schnelle Aktion erforderlich
    EMERGENCY,      // Notfall, sofortige Abschaltung
};

class SafetyMonitor {
public:
    SafetyMonitor();
    ~SafetyMonitor();
    
    /**
     * @brief Initialisiert Safety-Modul
     */
    bool init();
    
    /**
     * @brief Watchdog-Check (sollte regelmäßig aufgerufen werden)
     */
    void update();
    
    /**
     * @brief Prüft Sensor-Werte auf sichere Bereiche
     * @param accel_magnitude Größe der Beschleunigung (m/s²)
     * @param temperature Temperatur (°C)
     * @param altitude Höhe (m)
     * @return Safety-Level
     */
    SafetyLevel checkSensorBounds(float accel_magnitude, float temperature, float altitude);
    
    /**
     * @brief Prüft Stromversorgung
     * @return true wenn Spannung ausreichend
     */
    bool checkPowerSupply();
    
    /**
     * @brief Prüft Speicher-Verfügbarkeit
     * @return true wenn genug freier Speicher
     */
    bool checkMemory();
    
    /**
     * @brief Aktiviert Notfall-Modus
     * - Stoppt alle nicht-essentiellen Funktionen
     * - Schaltet sichere Geräte aus
     */
    void emergencyShutdown();
    
    /**
     * @brief Gibt aktuellen Safety-Level zurück
     */
    SafetyLevel getCurrentLevel() const;
    
    /**
     * @brief Setzt Schwellenwerte für Sensoren
     */
    void setAccelThreshold(float threshold);
    void setTemperatureThreshold(float min_temp, float max_temp);
    void setAltitudeThreshold(float max_altitude);

private:
    SafetyLevel current_level = SafetyLevel::NOMINAL;
    uint32_t last_check_time = 0;
    uint32_t watchdog_timeout = 5000;  // 5 Sekunden Watchdog
    
    // Schwellenwerte
    float accel_threshold = 50.0f;      // m/s² (ca. 5g)
    float temp_min = -40.0f;            // °C
    float temp_max = 85.0f;             // °C
    float altitude_max = 100000.0f;     // m (100 km)
    
    void onSafetyLevelChange(SafetyLevel new_level);
};
