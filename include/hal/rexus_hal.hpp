#pragma once

#include <cstdint>
#include <Arduino.h>
#include "pin_config.hpp"

/**
 * @file rexus_hal.hpp
 * @brief Hardware Abstraction Layer for REXUS Signal Interface
 * 
 * Provides interface for REXUS experiment signals:
 * - L0_t: Launch signal
 * - SOE_i: Start of Experiment
 * - SODS_i: Start of Descent
 */

/// Bitmaske der ROHEN Leitungszustaende fuer den SYS-Downlink (rawBits()).
static constexpr uint8_t DL_REXUS_L0   = 0x01;  ///< bit0: L0_t liegt HIGH an
static constexpr uint8_t DL_REXUS_SOE  = 0x02;  ///< bit1: SOE_i liegt HIGH an
static constexpr uint8_t DL_REXUS_SODS = 0x04;  ///< bit2: SODS_i liegt HIGH an

class REXUSHAL {
public:
    /**
     * @brief Wie lange ein Pegel stabil anliegen muss, bevor er als Signal gilt.
     *
     * Die Zustandsmaschine rastet auf diesen Signalen unumkehrbar ein
     * (PRE_LAUNCH --SODS--> ARMED). Ohne Entprellung genuegt EIN einzelner
     * verrauschter Sample-Zeitpunkt auf einer offenen Leitung, um die
     * Flugsequenz auszuloesen - am Bodenaufbau passiert das binnen Sekunden.
     */
    static constexpr uint32_t SIGNAL_STABLE_MS = 50;

    /**
     * @brief Constructor for REXUS HAL
     * @param pin_l0_t Launch signal pin
     * @param pin_soe_i Start of Experiment pin
     * @param pin_sods_i Start of Descent pin
     */
    REXUSHAL(uint8_t pin_l0_t, uint8_t pin_soe_i, uint8_t pin_sods_i);

    /**
     * @brief Initialize REXUS signal pins
     * @return true if successful
     */
    bool init();

    /**
     * @brief Leitungen sampeln und entprellen - einmal pro Loop aufrufen.
     *
     * Muss VOR getAllSignals()/read*() laufen, sonst arbeiten die Leser auf
     * dem Stand des vorigen Durchlaufs.
     */
    void update(uint32_t now_ms);

    /**
     * @brief Rohe (nicht entprellte) PIN-Pegel als Bitmaske, siehe DL_REXUS_*.
     *
     * Bewusst der elektrische Pegel, NICHT das interpretierte Signal: nur so
     * ist am Boden zu sehen, ob eine Leitung ruht oder anliegt - unabhaengig
     * davon, ob REXUS_ACTIVE_HIGH gerade richtig eingestellt ist.
     */
    uint8_t rawBits() const;

    /**
     * @brief Read L0_t signal
     * @return true if signal is active
     */
    bool readL0T();
    
    /**
     * @brief Read SOE_i signal
     * @return true if signal is active
     */
    bool readSOE();
    
    /**
     * @brief Read SODS_i signal
     * @return true if signal is active
     */
    bool readSODS();
    
    /**
     * @brief Get all signal states
     * @param l0_t Reference to L0_t state
     * @param soe Reference to SOE state
     * @param sods Reference to SODS state
     */
    void getAllSignals(bool& l0_t, bool& soe, bool& sods);
    
    /**
     * @brief Check if experiment has started (L0_t signal)
     * @return true if launched
     */
    bool isLaunched();
    
    /**
     * @brief Check if experiment data collection has started
     * @return true if experiment active
     */
    bool isExperimentActive();
    
    /**
     * @brief Check if spacecraft is descending
     * @return true if descending
     */
    bool isDescending();
    
private:
    /// Entprellzustand einer Leitung.
    struct Debounced {
        bool     stable   = false;  ///< als Signal gemeldeter Pegel
        bool     last_raw = false;  ///< zuletzt gesampelter Rohpegel
        uint32_t since_ms = 0;      ///< seit wann last_raw unveraendert anliegt
        bool     raw      = false;  ///< aktueller Rohpegel (fuer rawBits)
    };

    void sample(Debounced& d, bool raw, uint32_t now_ms);

    /**
     * @brief Setzt den Entprellzustand auf den tatsaechlich anliegenden Pegel.
     *
     * Muss in init() passieren. Ohne das startet `stable` auf LOW - bei
     * aktiv-LOW-Signalen hiesse das "ausgeloest", und die Zustandsmaschine
     * wuerde noch vor Ablauf des ersten Stabilitaetsfensters unumkehrbar nach
     * ARMED springen.
     */
    void seed(Debounced& d, uint8_t pin, uint32_t now_ms);

    uint8_t pin_l0_t_;
    uint8_t pin_soe_i_;
    uint8_t pin_sods_i_;
    bool initialized_ = false;

    Debounced l0_;
    Debounced soe_;
    Debounced sods_;
};
