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
 *   7..8  checkpoint_s - verstrichene Flugzeit in s (siehe StateMachine::elapsed)
 *   9..10 ~checkpoint_s
 *   11    flight_flags - welche HDRM-Fahrten schon begonnen wurden (FLIGHT_*)
 *   12    ~flight_flags
 *   13    arm_enable   - Flug-Schalter des Roboterarms (0x01 = an)
 *   14    ~arm_enable
 *   15..18 arm_step A  - Schritt der Flug-Trajektorie + Pruefwert (je uint16)
 *   19..22 arm_step B  - zweites Fach, abwechselnd beschrieben
 *   23    hdrm_blind   - Bit i = HDRM i+1 ohne Encoder-Bestaetigung "offen"
 *   24    ~hdrm_blind
 *
 * Jeder Wert hat ein invertiertes Pruefbyte. Ein ungueltiger Wert (nie
 * beschrieben, Stromausfall mitten im Schreiben) liest sich als 0.
 *
 * ACHTUNG Laufzeit: Jeder Flash-Schreibzugriff sperrt die Interrupts - normal
 * fuer Mikrosekunden, bei einem faelligen Sektor-Loeschen (selten) fuer
 * 45..400 ms. Waehrenddessen zaehlt kein Encoder. Deshalb wird NIE
 * geschrieben, waehrend ein Motor laeuft (siehe System), und nur Bytes, die
 * sich wirklich aendern.
 */

/// Bits in flight_flags: diese Fahrt wurde begonnen (auch wenn ein Neustart
/// sie unterbrochen hat). Jede HDRM-Fahrt laeuft hoechstens EINMAL pro Flug.
static constexpr uint8_t FLIGHT_HDRM_OPENED = 0x01;   ///< beide HDRMs: Oeffnen begonnen
static constexpr uint8_t FLIGHT_HDRM_CLOSED = 0x02;   ///< beide HDRMs: Schliessen begonnen
/// Flug-Trajektorie des Arms gestartet - nach einem Neustart fortsetzen statt
/// neu starten (ArmFlightPlayer::resume).
static constexpr uint8_t FLIGHT_ARM_STARTED = 0x04;
/// Flug-Trajektorie fertig - laeuft nie ein zweites Mal.
static constexpr uint8_t FLIGHT_ARM_DONE    = 0x08;
/// Beide HDRMs NACHWEISLICH offen (Encoder am Ziel) - erst dann faehrt der Arm.
static constexpr uint8_t FLIGHT_HDRM_OPEN_OK = 0x80;

/// Bits in flight_flags: dieses REXUS-Signal lag bei der Rueckkehr nach
/// PRE_LAUNCH noch an und zaehlt erst wieder, nachdem es einmal abgefallen ist
/// (siehe System::handleStateMachine).
static constexpr uint8_t FLIGHT_IGNORE_SODS = 0x10;
static constexpr uint8_t FLIGHT_IGNORE_LO   = 0x20;
static constexpr uint8_t FLIGHT_IGNORE_SOE  = 0x40;
static constexpr uint8_t FLIGHT_IGNORE_MASK = FLIGHT_IGNORE_SODS | FLIGHT_IGNORE_LO | FLIGHT_IGNORE_SOE;

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

    /// Verstrichene Flugzeit in vollen Sekunden (0, wenn ungueltig).
    uint16_t loadCheckpoint() const;
    void saveCheckpoint(uint16_t seconds);

    /// FLIGHT_*-Bits (0, wenn ungueltig).
    uint8_t loadFlightFlags() const;
    void saveFlightFlags(uint8_t flags);

    /// HDRMs, die nur auf Zeit / ohne Motor als offen gelten (Bit 0 = HDRM 1).
    /// Dann bricht der Arm ab, wenn er nicht vorankommt. 0, wenn ungueltig.
    uint8_t loadHdrmBlind() const;
    void saveHdrmBlind(uint8_t mask);

    /// Flug-Schalter des Roboterarms (ARM_FLIGHT_ENABLE). false, wenn ungueltig.
    bool loadArmFlightEnable() const;
    void saveArmFlightEnable(bool enabled);

    /// Zuletzt gesicherter Schritt der Flug-Trajektorie. false, wenn beide
    /// Faecher ungueltig sind.
    bool loadArmStep(uint16_t& step) const;
    void saveArmStep(uint16_t step);
    /// Beide Faecher auf 0 - beim Start der Flug-Trajektorie.
    void resetArmStep();

private:
    uint32_t boot_count_ = 0;
    bool     arm_step_slot_b_ = false;   ///< naechstes Fach fuer saveArmStep
};
