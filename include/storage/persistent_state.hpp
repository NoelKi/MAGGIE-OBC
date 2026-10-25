#pragma once

#include <cstdint>

/**
 * @file persistent_state.hpp
 * @brief Nichtfluechtiger Speicher fuer Zustand und Boot-Zaehler
 *
 * Liegt in der EEPROM-Emulation des Teensy (Flash) und uebersteht damit
 * Watchdog-Reset, Stromausfall UND das Neu-Flashen der Firmware.
 *
 * Layout (Bytes):
 *   0     MAGIC        - erkennt einen nie beschriebenen Speicher (0xFF)
 *   1..4  boot_count   - little-endian, +1 bei jedem Start
 *   5     state        - zuletzt erreichter MissionState
 *   6     ~state       - Pruefbyte: passt es nicht, gilt der Zustand als
 *                        ungueltig (halb geschrieben, nie beschrieben)
 *
 * ACHTUNG Laufzeit: Jeder Flash-Schreibzugriff sperrt die Interrupts - normal
 * fuer Mikrosekunden, bei einem faelligen Sektor-Loeschen (ca. alle 1000
 * Schreibzugriffe) fuer 45..400 ms. Deshalb wird NUR beim Booten und bei
 * Zustandswechseln geschrieben, und nur Bytes, die sich wirklich aendern.
 */
class PersistentState {
public:
    /// Speicher lesen und den Boot-Zaehler hochzaehlen. Einmal beim Start.
    void begin();

    /// Nummer dieses Starts (1 = erster Start mit leerem Speicher).
    uint32_t bootCount() const { return boot_count_; }

    /**
     * @brief Zuletzt gespeicherten Zustand lesen.
     * @return false, wenn nie ein Zustand geschrieben wurde oder er beschaedigt ist
     */
    bool loadState(uint8_t& state) const;

    /// Zustand speichern. Nur bei Zustandswechseln aufrufen (siehe oben).
    void saveState(uint8_t state);

private:
    uint32_t boot_count_ = 0;
};
