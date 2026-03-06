#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Ground Test Controller
 * Zum Testen einzelner Hardware-Komponenten am Boden
 * 
 * Funktionen:
 * - Test einzelner Geräte/Sensoren
 * - Manuelle Steuerung von Ventilen, Motoren, etc.
 * - Kalibrierung und Hardware-Validierung
 * - KEINE komplexe Experiment-Logik
 */

enum class GroundTestMode {
    IDLE,                // Bereit
    SENSOR_TEST,         // Sensoren testen (IMU, Druck, Temperatur, etc.)
    DEVICE_ACTIVATION,   // Geräte aktivieren (Ventile, Pumpen, Motor)
    CALIBRATION,         // Sensorkalibrierung
    CONTINUOUS_READ,     // Kontinuierliche Sensorwerte auslesen
};

class GroundTestController {
public:
    GroundTestController();
    ~GroundTestController();
    
    /**
     * @brief Initialisiert Ground Test Controller
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Update-Loop für Ground Tests
     */
    void update();
    
    /**
     * @brief Teste spezifischen Sensor
     * @param sensor_id Sensor-ID (0=IMU, 1=Barometer, 2=Temp, etc.)
     */
    void testSensor(uint8_t sensor_id);
    
    /**
     * @brief Teste spezifisches Gerät
     * @param device_id Geräte-ID (0=Ventil1, 1=Ventil2, 2=Motor, etc.)
     */
    void testDevice(uint8_t device_id);
    
    /**
     * @brief Aktiviere Gerät manuell
     * @param device_id Geräte-ID
     * @param power PWM-Wert (0-255)
     */
    void activateDevice(uint8_t device_id, uint8_t power = 255);
    
    /**
     * @brief Deaktiviere Gerät
     * @param device_id Geräte-ID
     */
    void deactivateDevice(uint8_t device_id);
    
    /**
     * @brief Starte Sensorkalibrierung
     * @param sensor_id Sensor-ID zu kalibrieren
     */
    void calibrateSensor(uint8_t sensor_id);
    
    /**
     * @brief Starte kontinuierliche Sensorüberwachung
     * @param sensor_id Sensor-ID
     */
    void startContinuousRead(uint8_t sensor_id);
    
    /**
     * @brief Stoppe kontinuierliche Überwachung
     */
    void stopContinuousRead();
    
    /**
     * @brief Gibt aktuellen Test-Status zurück
     */
    GroundTestMode getMode() const;
    
    /**
     * @brief Serial-Befehl Handler (für Debug-Konsole)
     * @param command String-Befehl (z.B. "TEST_SENSOR 0", "ACTIVATE_DEVICE 1 200")
     */
    void handleSerialCommand(const String& command);

private:
    GroundTestMode current_mode = GroundTestMode::IDLE;
    uint8_t active_sensor = 0xFF;
    uint8_t active_device = 0xFF;
    uint32_t last_read_time = 0;
    
    void printSensorData(uint8_t sensor_id);
    void printDeviceStatus(uint8_t device_id);
    void setMode(GroundTestMode new_mode);
};
