#pragma once

#include <cstdint>
#include <Arduino.h>
#include <IntervalTimer.h>
#include <Encoder.h>

/**
 * @file motor_hal.hpp
 * @brief Hardware Abstraction Layer for Motor Control (DRV8871)
 *
 * Reine Steuerung (Open Loop) einer DRV8871-H-Bruecke ueber die Kanaele A/B.
 * Der Quadratur-Encoder haengt optional daran, wird aber NUR als Sensor
 * gelesen - es gibt keine Positionsregelung. Ein einmal gestarteter Motor
 * dreht, bis off() kommt.
 *
 * Die frueheren Fahrten auf Encoder-Ziel (moveTo/halfTurn/update mit P-Regler,
 * Stall- und Zeitueberwachung) sind entfallen: Der Bodentest soll die
 * Telecommand-Strecke und den Motor selbst pruefen, nicht die Regelung. Der
 * Encoder ist damit Messmittel statt Regelgroesse.
 */

class MotorHAL {
public:
    /// Counts pro voller Umdrehung der Abtriebswelle.
    /// Hergeleitet aus der Bestueckung: Der 12-CPR-Encoder sitzt auf der
    /// MOTORwelle, das Getriebe untersetzt 379.17:1 -> 12 x 379.17 = 4550.04.
    /// Basis fuer die Winkelanzeige am Boden und fuer turnBy()/goTo().
    /// Gegenstueck: MAGGIE_SERVER/app/services/downlink_frame_parser.py.
    static constexpr long COUNTS_PER_REV = 4550;

    /// Kleinster PWM-Betrag, mit dem der Motor ueberhaupt anlaeuft.
    ///
    /// Am Aufbau gemessen: Unter etwa 150 dreht er sich selbst OHNE Last nicht
    /// mehr, er brummt nur. Das ist keine Kennlinienschwaeche, sondern
    /// Haftreibung plus Getriebewiderstand - und unter Last wird die Schwelle
    /// eher hoeher, nie niedriger. 220 haelt bewusst Abstand dazu.
    ///
    /// turnBy()/goTo() fahren nie mit weniger als diesem Wert (siehe
    /// TURN_SPEED) - ein Fahrbefehl auf Encoder-Ziel braucht die Anlaufschwelle,
    /// sonst liefe der Stillstands-Abbruch sofort los, ohne dass sich etwas
    /// bewegt hat.
    ///
    /// setSpeed() selbst hebt seit dem Bodentest vom 2026-08-26 NICHT mehr an
    /// (siehe motor_hal.cpp) - der manuelle Dauerlauf (on()/MOTOR_ON) darf
    /// bewusst auch darunter fahren, um die reale Anlaufschwelle zu vermessen.
    static constexpr int16_t MIN_DRIVE_SPEED = 220;

    /**
     * @brief Constructor for Motor HAL
     * @param pin_a Channel A (PWM)
     * @param pin_b Channel B (PWM)
     * @param motor_id Motor identifier (1-3)
     * @param soft_approach Sanfte Anfahrt fuer turnBy()/goTo() (siehe unten,
     *                      SOFT_APPROACH_*). Bewusst per Instanz statt global,
     *                      damit Motor 1 unveraendert bleibt.
     */
    MotorHAL(uint8_t pin_a, uint8_t pin_b, uint8_t motor_id = 0, bool soft_approach = false);
    ~MotorHAL();

    /**
     * @brief Initialize motor pins (open-loop)
     * @return true if successful
     */
    bool init();

    /**
     * @brief Quadratur-Encoder anhängen (reine Positionsmessung)
     * @param pin_enc_a Encoder Channel A
     * @param pin_enc_b Encoder Channel B
     * @return true if successful
     */
    bool initEncoder(uint8_t pin_enc_a, uint8_t pin_enc_b);

    /**
     * @brief Set motor speed and direction
     * @param speed -255 (full reverse) to +255 (full forward), 0 = stop
     *
     * Betraege zwischen 1 und MIN_DRIVE_SPEED werden auf MIN_DRIVE_SPEED
     * angehoben - darunter laeuft der Motor nicht an. 0 bleibt 0.
     */
    void setSpeed(int16_t speed);

    /**
     * @brief Stop motor immediately (Coast - der Motor laeuft frei aus)
     */
    void stop();

    /**
     * @brief Aktiv bremsen statt auslaufen lassen.
     *
     * Schaltet beide Kanaele HIGH. Laut DRV8871-Datenblatt (Table 1,
     * H-Bridge Control) ist 1/1 "Brake; low-side slow decay" - die
     * Motorklemmen werden kurzgeschlossen. 0/0 waere dagegen "Coast", also
     * freier Auslauf; das ist die Hauptquelle des Ueberschwingens am Zielpunkt.
     *
     * Der Bremsimpuls endet nach BRAKE_MS automatisch (siehe update()), damit
     * die H-Bruecke danach schlafen kann und sich die Mechanik von Hand
     * bewegen laesst.
     */
    void brake();

    /**
     * @brief Get current motor speed
     * @return Current speed value (-255..255)
     */
    int16_t getSpeed() const { return current_speed_; }

    /**
     * @brief Set PWM frequency
     * @param frequency Frequency in Hz
     *
     * Im Software-PWM-Modus (siehe usesSoftPwm) wird der Wert auf
     * SOFT_PWM_HZ_MIN..SOFT_PWM_HZ_MAX begrenzt - 20 kHz sind per
     * IntervalTimer nicht erreichbar.
     */
    void setPWMFrequency(uint32_t frequency);

    /**
     * @brief true, wenn dieser Motor per Software-PWM getaktet wird.
     *
     * Greift automatisch, sobald einer der beiden Kanaele auf einem Pin ohne
     * PWM-Timer liegt (z.B. 40/41 auf der Teensy 4.1). Mit der aktuellen
     * Belegung (18/19, beide QuadTimer) bleibt der Modus aus.
     */
    bool usesSoftPwm() const { return soft_pwm_; }

    /**
     * @brief Hat dieser Teensy-4.1-Pin einen FlexPWM-/QuadTimer-Kanal?
     *
     * Auf Pins ohne Eintrag in pwm_pin_info[] kehrt analogWrite() wirkungslos
     * zurueck (cores/teensy4/pwm.c: `else { return; }`) - ohne Fehlermeldung.
     */
    static bool pinHasHardwarePwm(uint8_t pin);

    // -----------------------------------------------------------------------
    // Encoder (reine Messung, siehe initEncoder)
    // -----------------------------------------------------------------------

    /** @brief Aktuelle Encoder-Position in Quadratur-Counts (0 ohne Encoder). */
    long getPosition();

    /**
     * @brief Setzt den Encoder-Zaehler auf 0 und stoppt den Motor.
     *
     * Keine Regelung - nur ein Nullpunkt fuer die Anzeige am Boden. Damit
     * laesst sich COUNTS_PER_REV am Tisch nachmessen: nullen, eine
     * Wellenumdrehung drehen lassen, Counts ablesen.
     */
    void zeroPosition();

    /**
     * @brief Motor dauerhaft drehen lassen.
     * @param speed -255..+255, Vorzeichen = Drehrichtung.
     *              0 nimmt DEFAULT_ON_SPEED (vorwaerts).
     */
    void on(int16_t speed = 0);

    /** @brief Motor ausschalten (bricht eine laufende turnBy()-Drehung ab). */
    void off();

    /**
     * @brief Dreht um einen festen Winkel und stoppt am Ziel.
     * @param degrees Drehwinkel, Vorzeichen = Richtung. 0 tut nichts.
     *
     * KEINE Regelung: Der Motor laeuft mit konstantem TURN_SPEED, und
     * updateTurn() schaltet ihn ab, sobald der Encoder die Zielcounts
     * ueberschritten hat - der Encoder wirkt also als Endschalter, nicht als
     * Regelgroesse. Es wird weder die Geschwindigkeit nachgefuehrt noch am Ziel
     * nachkorrigiert; der Auslauf bleibt als Restfehler stehen und ist in der
     * Telemetrie sichtbar.
     *
     * Ohne Encoder passiert nichts (sonst liefe der Motor ungebremst weiter).
     */
    void turnBy(int16_t degrees);

    /**
     * @brief Faehrt auf einen ABSOLUTEN Winkel bezogen auf die Encoder-Null.
     * @param degrees Zielwinkel; 0 ist die mit zeroPosition() gesetzte Nullage.
     *
     * Der Unterschied zu turnBy() ist der Grund, warum es beide gibt: turnBy()
     * rechnet ab der aktuellen Position und erbt damit den Fehler jeder
     * vorherigen Fahrt - bei wiederholtem Auf/Zu addiert sich der
     * richtungsabhaengige Ueberschwinger auf und die Nullage wandert. goTo()
     * bezieht sich immer auf denselben Nullpunkt, der Fehler bleibt dadurch
     * beschraenkt statt zu akkumulieren.
     *
     * Liegt die Position bereits innerhalb von POS_DEADBAND um das Ziel,
     * passiert nichts - sonst wuerde der Motor um den Zielpunkt pendeln.
     */
    void goTo(int16_t degrees);

    /**
     * @brief Zyklische Pflege - muss jeden Loop aufgerufen werden.
     *
     * Schaltet eine laufende Fahrt am Ziel ab (siehe turnBy/goTo) und beendet
     * den Bremsimpuls nach BRAKE_MS. Ohne laufende Fahrt und ohne Bremsung
     * kehrt die Funktion sofort zurueck. Das ist KEIN Regler.
     */
    void update();

    bool isOn() const { return is_on_; }          ///< Dauer-An/Aus-Zustand (on()/off())
    bool isTurning() const { return turning_; }   ///< turnBy()-Drehung laeuft
    bool hasEncoder() const { return enc_ != nullptr; }

    /**
     * @brief Wurde die letzte Drehung abgebrochen, statt das Ziel zu erreichen?
     *
     * Wird bei jedem turnBy() zurueckgesetzt. Siehe TURN_STALL_MS - typische
     * Ursachen sind ein nicht zaehlender oder verpolter Encoder.
     */
    bool turnFailed() const { return turn_failed_; }

private:
    uint8_t pin_a_;
    uint8_t pin_b_;
    uint8_t motor_id_;
    bool soft_approach_ = false;   ///< siehe SOFT_APPROACH_* oben
    int16_t current_speed_ = 0;
    bool initialized_ = false;

    Encoder* enc_ = nullptr;   ///< Quadratur-Encoder (nullptr = keine Messung)
    bool is_on_  = false;      ///< logischer An/Aus-Zustand (on()/off())

    bool turning_     = false; ///< eine Fahrt auf ein Encoder-Ziel laeuft
    long turn_start_  = 0;     ///< Position beim Start dieser Fahrt
    long turn_target_ = 0;     ///< absolute Zielposition dieser Fahrt in Counts
    bool turn_failed_ = false; ///< letzte Fahrt wurde abgebrochen

    // Ueberwachung der laufenden Fahrt, siehe updateTurn().
    uint32_t turn_ref_ms_  = 0;  ///< Beginn des aktuellen Fortschrittsfensters
    long     turn_ref_pos_ = 0;  ///< Position zu Beginn dieses Fensters

    bool     braking_        = false;  ///< Bremsimpuls laeuft
    uint32_t brake_start_ms_ = 0;      ///< Beginn des Bremsimpulses

    /// Gemeinsamer Start fuer turnBy() und goTo().
    void startMove(long target, int16_t degrees, const char* kind);
    void updateTurn();   ///< Fahrt am Ziel abschalten
    void updateBrake();  ///< Bremsimpuls nach BRAKE_MS beenden

    // -----------------------------------------------------------------------
    // Software-PWM fuer Pins ohne Hardware-Timer
    // -----------------------------------------------------------------------
    // Ein IntervalTimer tickt SOFT_PWM_STEPS mal pro Periode und setzt die
    // beiden Kanaele per digitalWrite. Die Statics gelten fuer EINEN Motor -
    // aktuell wird auch nur Motor 1 instanziiert (system.cpp).
    static constexpr uint32_t SOFT_PWM_STEPS  = 256;   ///< Aufloesung = analogWrite-Bereich
    static constexpr uint32_t SOFT_PWM_HZ_DEF = 1000;  ///< Default-Traegerfrequenz
    static constexpr uint32_t SOFT_PWM_HZ_MIN = 100;
    static constexpr uint32_t SOFT_PWM_HZ_MAX = 2000;  ///< darueber wird die ISR-Last zu hoch
    /// Interrupt-Prioritaet: hoeherer Wert = niedrigere Prioritaet. Muss unter
    /// der des Encoders (Default 128) liegen, sonst gehen Quadratur-Flanken
    /// verloren und die Positionsregelung driftet.
    static constexpr uint8_t  SOFT_PWM_PRIORITY = 192;

    bool soft_pwm_ = false;    ///< dieser Motor laeuft auf Software-PWM

    static IntervalTimer   soft_timer_;
    static volatile uint8_t soft_duty_a_;
    static volatile uint8_t soft_duty_b_;
    static volatile uint8_t soft_tick_;
    static uint8_t soft_pin_a_;
    static uint8_t soft_pin_b_;
    static bool    soft_active_;

    static void softPwmIsr();
    void softPwmBegin(uint32_t frequency);

    /** @brief Schreibt beide Kanaele - je nach Modus per analogWrite oder Soft-PWM. */
    void writeChannels(uint8_t duty_a, uint8_t duty_b);

    /// Geschwindigkeit fuer on() ohne Argument.
    static constexpr int16_t DEFAULT_ON_SPEED = MIN_DRIVE_SPEED;

    /// Feste Geschwindigkeit fuer turnBy()/goTo(). Bewusst NICHT vom
    /// PWM-Schieber der Bodenstation abhaengig: Der Nachlauf am Ziel haengt an
    /// der Drehzahl, mit konstantem Wert ist er reproduzierbar.
    ///
    /// Nach unten ist hier kein Spielraum - MIN_DRIVE_SPEED ist die Grenze,
    /// unter der der Motor gar nicht erst anlaeuft. Der Nachlauf laesst sich
    /// also nicht ueber die Drehzahl verkleinern; dagegen arbeiten der
    /// Bremsimpuls (siehe brake()) und POS_DEADBAND.
    static constexpr int16_t TURN_SPEED = MIN_DRIVE_SPEED;

    // -----------------------------------------------------------------------
    // Abbruchkriterium fuer turnBy()
    // -----------------------------------------------------------------------
    // Ohne diese Grenze laeuft der Motor bis zum Laufzeit-Watchdog in System
    // (30 s), sobald das Ziel unerreichbar ist - bei totem Encoder, verpolten
    // Kanaelen oder blockierter Mechanik. Das sind dutzende Umdrehungen, bevor
    // ueberhaupt auffaellt, dass etwas nicht stimmt.
    static constexpr uint32_t TURN_STALL_MS    = 1000;  ///< Fenster ohne Fortschritt
    static constexpr long     TURN_MIN_COUNTS  = 3;     ///< Fortschritt, der als Bewegung zaehlt

    // -----------------------------------------------------------------------
    // Sanfte Anfahrt (nur wenn soft_approach_ true ist, siehe Konstruktor)
    // -----------------------------------------------------------------------
    // Hintergrund: TURN_SPEED == MIN_DRIVE_SPEED, es gibt also normalerweise
    // GAR KEINE Drehzahlreserve fuer eine Rampe - unter MIN_DRIVE_SPEED laeuft
    // der Motor gar nicht erst an. soft_approach_ nutzt deshalb die Reserve
    // NACH OBEN: cruist mit SOFT_APPROACH_CRUISE_SPEED (> TURN_SPEED) und
    // bremst erst kurz vor dem Ziel auf SOFT_APPROACH_SPEED herunter - weniger
    // Aufprallenergie an einem mechanischen Anschlag, ohne dass der Motor
    // vorher stehenbleibt.
    //
    // SOFT_APPROACH_SPEED = 170 (Bodentest 2026-08-26): Der Bodentest mit
    // frei zugaenglichem PWM-Bereich (siehe MOTOR_ON) hat die reale, lastfreie
    // Anlaufschwelle bei ~150 bestaetigt (siehe MIN_DRIVE_SPEED-Kommentar).
    // 170 haelt etwas Abstand darueber, ist aber spuerbar sanfter als die
    // bisherigen 220 (MIN_DRIVE_SPEED) fuer die Anfahrt selbst. MIN_DRIVE_SPEED
    // bleibt unveraendert der Wert fuer TURN_SPEED/DEFAULT_ON_SPEED - dort
    // zaehlt zuverlaessiges Anlaufen unter Last mehr als Sanftheit.
    //
    // Zusaetzlich gilt in dieser Anfahrzone ein VIEL kuerzeres Stall-Fenster:
    // Ein Vorfall (Motor 2, siehe Command-Log) fuhr mit vollem TURN_SPEED
    // gegen einen Anschlag und stand die vollen TURN_STALL_MS mit
    // Blockierstrom an, bevor der Watchdog abschaltete - plausibel genug, um
    // den Teensy per Spannungseinbruch zu resetten. In der Anfahrzone wird
    // "kein Fortschritt mehr" deshalb nicht als Fehler gewertet, sondern als
    // ERREICHTES Ziel (Kontakt/Anschlag) interpretiert - turn_failed_ bleibt
    // false, der Motor schaltet trotzdem sofort ab.
    static constexpr long     SOFT_APPROACH_WINDOW_COUNTS = 300;  // ~24 Grad bei 4550 Counts/U
    static constexpr uint32_t SOFT_APPROACH_STALL_MS       = 150;
    static constexpr int16_t  SOFT_APPROACH_CRUISE_SPEED   = 255;
    static constexpr int16_t  SOFT_APPROACH_SPEED          = 170;

    /// Dauer des Bremsimpulses am Ende einer Fahrt. Lang genug, damit der
    /// Anker steht, kurz genug, dass die H-Bruecke danach wieder schlafen kann.
    static constexpr uint32_t BRAKE_MS = 250;

    /// Zielfenster fuer goTo().
    ///
    /// MUSS groesser sein als der Nachlauf nach dem Bremsen. Eine Fahrt endet
    /// immer ein Stueck HINTER dem Ziel; ist das Fenster kleiner als dieser
    /// Rest, sieht ein erneuter Befehl auf dieselbe Position eine Abweichung
    /// und faehrt zurueck - beim naechsten Druck wieder vor. Der Motor pendelt
    /// dann um den Zielpunkt, statt stehen zu bleiben.
    ///
    /// Einstellen: Nachlauf ablesen (die "Fahrt beendet"-Zeile nennt Position
    /// und Ziel, die Differenz ist der Nachlauf) und rund das Anderthalbfache
    /// davon eintragen. 200 Counts sind ~15.8 Grad bei 4550 Counts/Umdrehung -
    /// bewusst grosszuegig, weil ein zu kleines Fenster schlimmer ist als ein
    /// zu grosses: Die Genauigkeit der Endlage leidet nur um diesen Betrag,
    /// waehrend Pendeln die Mechanik belastet.
    static constexpr long POS_DEADBAND = 200;
};
