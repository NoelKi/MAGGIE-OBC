#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * @brief Speicher-Treiber für SD-Karte
 * Verwendet SPI für Datenzugriff
 * Speichert Telemetrie-Daten auf SD-Karte
 */

class StorageDriver {
public:
    StorageDriver();
    ~StorageDriver();
    
    /**
     * @brief Initialisiert SD-Karte
     * @return true wenn erfolgreich
     */
    bool init();
    
    /**
     * @brief Schreibt Daten in Datei
     * @param filename Name der Datei
     * @param data Zeiger auf Daten
     * @param length Anzahl Bytes
     * @return Bytes geschrieben
     */
    uint32_t write(const char* filename, const void* data, uint32_t length);
    
    /**
     * @brief Liest Daten aus Datei
     * @param filename Name der Datei
     * @param data Zeiger auf Buffer
     * @param length Max. zu lesende Bytes
     * @return Bytes gelesen
     */
    uint32_t read(const char* filename, void* data, uint32_t length);
    
    /**
     * @brief Erstellt CSV-Datei mit Header
     * @param filename Name der Datei
     * @param header CSV Header
     */
    bool createCSVFile(const char* filename, const char* header);
    
    /**
     * @brief Fügt Zeile zu CSV-Datei hinzu
     * @param filename Name der Datei
     * @param row CSV Zeile
     */
    bool appendCSVRow(const char* filename, const char* row);
    
    /**
     * @brief Gibt verfügbaren Speicher zurück
     */
    uint64_t getFreeSpace();
    
    /**
     * @brief Prüft Speicher-Zustand
     */
    bool isHealthy() const;

private:
    bool healthy = false;
    static constexpr uint8_t CS_PIN = 10;  // Chip Select
};
