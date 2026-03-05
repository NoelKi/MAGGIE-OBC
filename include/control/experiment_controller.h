#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Experiment-Kontroll-Modul
 * Steuert das spezifische Experiment (z.B. Datenerfassung,
 * Ventilsteuerung, Mechanismus-Aktivierung, etc.)
 */

enum class ExperimentState {
    IDLE,           // Inaktiv, auf Start warten
    ARMED,          // Bereit zu starten
    RUNNING,        // Laufend
    CALIBRATING,    // Kalibrierung läuft
    PAUSED,         // Pausiert
    FINISHED,       // Abgeschlossen
    ERROR,          // Fehler aufgetreten
};

class ExperimentController {
public:
    ExperimentController();
    ~ExperimentController();
    
    /**
     * @brief Initialisiert das Experiment
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Startet das Experiment
     * @return true wenn erfolgreich
     */
    bool start();
    
    /**
     * @brief Pausiert das Experiment
     */
    bool pause();
    
    /**
     * @brief Setzt das Experiment fort
     */
    bool resume();
    
    /**
     * @brief Stoppt und setzt das Experiment zurück
     */
    void reset();
    
    /**
     * @brief Hauptkontroll-Loop (sollte regelmäßig aufgerufen werden)
     */
    void update();
    
    /**
     * @brief Gibt aktuellen Zustand zurück
     */
    ExperimentState getState() const;
    
    /**
     * @brief Gibt Dauer des aktuellen Experiments zurück
     */
    uint32_t getDuration() const;
    
    /**
     * @brief Kalibriert Sensoren vor Start
     */
    void calibrateSensors();
    
    /**
     * @brief Aktiviert externes Gerät/Ventil (z.B. Pump, Solenoid)
     * @param device_id Geräte-ID
     * @return true wenn erfolgreich
     */
    bool activateDevice(uint8_t device_id);
    
    /**
     * @brief Deaktiviert externes Gerät
     */
    bool deactivateDevice(uint8_t device_id);

private:
    ExperimentState current_state = ExperimentState::IDLE;
    uint32_t start_time = 0;
    uint32_t pause_start_time = 0;
    uint32_t total_paused_time = 0;
    
    static constexpr uint32_t MAX_EXPERIMENT_DURATION = 300000;  // 5 Minuten Max
    
    void onStateChange(ExperimentState new_state);
};
