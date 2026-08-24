#pragma once

#include <cstdint>
#include <Arduino.h>

/**
 * @file force_hal.hpp
 * @brief Hardware Abstraction Layer fuer die Kraftsensoren (HX711-Gruppen)
 *
 * Ein Kraftsensor ist hier eine Gruppe von HX711-Wandlern an EINER gemeinsamen
 * Taktleitung. Am OBC haengen zwei davon (siehe pin_config.hpp):
 *
 *   Sensor 1 (Target 1): 3 Zellen X/Y/Z  an Pin 4/3/2,     Takt 27
 *   Sensor 2 (Target 2): 4 Zellen A..D   an Pin 30/31/32/41, Takt 9
 *
 * Der OBC liest beide gleich: rohe Counts, Nullpunkt abziehen, herunterfunken.
 * Was die Zellen bedeuten, entscheidet die Bodenstation - bei Sensor 2 stehen
 * drei Zellen um 120 Grad versetzt in der XY-Ebene und die vierte misst Z, aber
 * diese Verrechnung passiert bewusst NICHT hier (siehe unten).
 *
 * ---------------------------------------------------------------------------
 * Warum ein eigener Treiber statt der HX711-Bibliothek
 * ---------------------------------------------------------------------------
 * Die eingebundene Lib (bogde/HX711) haelt pro Instanz ein eigenes SCK-Pin und
 * taktet beim Lesen 25 Impulse. Liegen alle Wandler einer Gruppe am selben
 * Takt, sieht jeder dieser Impulse ALLE Chips: Beim Lesen von Kanal 0 schieben
 * die uebrigen ihre Messwerte ebenfalls heraus, nur liest sie niemand - sie
 * sind verloren, und der naechste Wert steht erst nach der naechsten Wandlung
 * (100 ms) an.
 *
 * Deshalb wird hier EINMAL getaktet und in derselben Schleife von allen
 * DOUT-Leitungen gelesen. Nebeneffekt: Die Werte einer Gruppe gehoeren zum
 * selben Zeitpunkt - bei einem Kraftvektor aus mehreren Zellen ist das kein
 * Luxus, sondern Voraussetzung. Bei Sensor 2 haengt die gesamte
 * XY-Verrechnung daran: Stammten A, B und C aus verschiedenen Wandlungen,
 * waere der berechnete Vektor waehrend jeder Laständerung falsch.
 *
 * ---------------------------------------------------------------------------
 * Interrupts waehrend der Taktschleife
 * ---------------------------------------------------------------------------
 * Der HX711 schaltet ab, wenn PD_SCK laenger als 60 us HIGH bleibt
 * (Datenblatt: T3 max 50 us). Die Schleife haelt HIGH ca. 1-2 us; ein
 * dazwischenfunkender Interrupt verlaengert das um seine Laufzeit.
 *
 * Die Interrupts bleiben trotzdem AN. Alle ISRs dieser Firmware (Encoder,
 * Serial4, USB, SysTick) liegen im einstelligen Mikrosekundenbereich und damit
 * eine Groessenordnung unter der Grenze. Ein `noInterrupts()` ueber die
 * gesamte Leseschleife waere ~60 us blind - und genau das gefaehrdet die
 * Quadratur-Flanken des Motor-Encoders, also einen Wert, den wir behalten
 * wollen. Kaeme spaeter eine lange ISR dazu (Software-PWM, DMA-Callback),
 * gehoert diese Abwaegung neu gemacht.
 */

/// Maximale Zellen pro Sensorgruppe. 4 reicht fuer beide verbauten Sensoren
/// und haelt ForceReading klein genug, um sie per Wert zu reichen.
static constexpr uint8_t FORCE_MAX_CHANNELS = 4;

/// Wandlungsrate des HX711 mit RATE-Pin auf LOW. Bestimmt, wie oft read()
/// ueberhaupt einen neuen Wert liefern kann.
static constexpr uint32_t FORCE_SAMPLE_RATE_HZ = 10;

/**
 * @brief Teiler zwischen tarierten Counts und dem int16 des Downlinks.
 *
 * Der HX711 liefert 24 Bit, das Downlink-Frame hat pro Kanal nur 16 - die
 * vollen Counts passen also nicht hinein. Statt stumpf die unteren Bits
 * wegzuwerfen (das kostet Aufloesung genau dort, wo gemessen wird) geht der
 * TARIERTE Wert nach unten: Der ruht um 0 herum, die interessante Amplitude
 * liegt also im int16-Fenster, und der grosse Nullpunkt-Offset des Wandlers
 * faellt vorher weg.
 *
 * JEDER Sensor hat seinen EIGENEN Teiler, weil die beiden Wandlergruppen
 * unterschiedliche Wiegezellen an unterschiedlichem Takt sind und damit
 * unterschiedlich viele Rohcounts pro Gramm liefern - ein gemeinsamer Teiler
 * kann nicht fuer beide gleichzeitig passen.
 *
 * ERSTKALIBRIERUNG (Bodentest 2026-08-24, vertikale Belastung):
 * Bei FORCE_TELE_DIV=1 saettigte Sensor 1 (X/Y/Z, 20 N = ca. 2039 g) schon
 * bei ca. 10 g - der Wandler liefert also ca. 32767/10 ≈ 3277 Rohcounts pro
 * Gramm. Um die vollen 2039 g mit Reserve abzubilden, braucht es einen
 * Teiler von mindestens 2039/10 ≈ 204; gewaehlt: 256 (Vollausschlag ≈ 2560 g,
 * ca. 25 % Reserve ueber der Nennlast).
 *
 * Sensor 2 (A/B/C/D, je Zelle 10 kg = 10000 g) wurde noch NICHT einzeln
 * durchgemessen. Nimmt man an, dass die Zellen bei aehnlicher Bauart auf
 * dieselbe Vollausschlags-Spannung (mV/V) ausgelegt sind, skalieren
 * Rohcounts/Gramm umgekehrt proportional zur Nennlast - der noetige Teiler
 * waere dann ungefaehr derselbe wie bei Sensor 1 (2039/10 ≈ 10000/49, beide
 * ≈ Faktor 204). Deshalb vorerst derselbe Wert (256) als Startpunkt.
 *
 * NACHMESSEN: Saettigt Sensor 2 bei bekannter Last trotzdem (DL_FORCE_SAT_*
 * im STATUS2, siehe unten), FORCE2_TELE_DIV verdoppeln, bis der Bereich
 * passt. Beim Aendern beider Werte IMMER MAGGIE_SERVER/app/services/
 * downlink_frame_parser.py (FORCE1_TELE_DIV / FORCE2_TELE_DIV) mitziehen -
 * sonst stimmt die Skalierung (Counts -> Newton) am Boden nicht mehr.
 */
static constexpr int32_t FORCE1_TELE_DIV = 256;   ///< Kraftsensor 1 (X/Y/Z, 20 N)
static constexpr int32_t FORCE2_TELE_DIV = 256;   ///< Kraftsensor 2 (A/B/C/D, 10 kg/Zelle) - Schaetzung, siehe oben

/// Ab wann gilt der Sensor als haengend (keine neue Wandlung mehr)?
/// Grosszuegig gegenueber den 100 ms der 10-Hz-Wandlung.
static constexpr uint32_t FORCE_STALE_MS = 500;

struct ForceReading {
    /// Belegte Kanaele (3 bei Sensor 1, 4 bei Sensor 2).
    uint8_t count = 0;

    // Rohe 24-Bit-Counts, vorzeichenerweitert. Nur fuer die Kalibrierung am
    // USB-Terminal interessant - sie gehen NICHT in den Downlink.
    int32_t raw[FORCE_MAX_CHANNELS] = {};

    // Tariert (raw minus Nullpunkt aus tare()). Das ist der physikalische Wert
    // bis auf den Kalibrierfaktor, den die Bodenstation anwendet.
    int32_t counts[FORCE_MAX_CHANNELS] = {};

    // Tariert, durch tele_div_ (FORCE1_/FORCE2_TELE_DIV) geteilt und auf int16 begrenzt.
    int16_t tele[FORCE_MAX_CHANNELS] = {};

    // Hat die Begrenzung gegriffen? Dann ist der Wert im Downlink abgeschnitten
    // und die Anzeige am Boden zu klein - nicht nur ungenau. Bei Sensor 2
    // faellt das doppelt ins Gewicht: Dort geht jede Zelle in die Verrechnung
    // von X und Y ein, eine still abgeschnittene verdreht also den GESAMTEN
    // Kraftvektor, nicht nur ihren eigenen Kanal.
    bool sat[FORCE_MAX_CHANNELS] = {};

    uint32_t timestamp = 0;
    bool valid = false;
};

class ForceHAL {
public:
    /// Samples fuer den Nullabgleich. 8 Stueck sind bei 10 Hz 0,8 s - lang
    /// genug, um das Rauschen herauszumitteln, kurz genug fuer den Bootvorgang.
    static constexpr uint8_t  TARE_SAMPLES    = 8;
    static constexpr uint32_t TARE_TIMEOUT_MS = 2000;

    /**
     * @brief Constructor
     * @param dout_pins Datenleitungen der Wandler, in Kanalreihenfolge
     * @param count     Anzahl Kanaele (1..FORCE_MAX_CHANNELS)
     * @param pin_sck   GEMEINSAME Taktleitung aller Wandler dieser Gruppe
     * @param tele_div  Teiler tarierte Counts -> int16 (siehe FORCE1_TELE_DIV /
     *                  FORCE2_TELE_DIV oben) - je Sensor unterschiedlich, weil
     *                  die Wiegezellen unterschiedlich viele Counts/Gramm liefern.
     *
     * Die Reihenfolge in dout_pins IST die Kanalnummerierung im Downlink -
     * bei Sensor 2 also A, B, C, D. Wird sie vertauscht, dreht sich am Boden
     * der berechnete Kraftvektor mit.
     */
    ForceHAL(const uint8_t* dout_pins, uint8_t count, uint8_t pin_sck, int32_t tele_div);

    /**
     * @brief Pins konfigurieren und Nullabgleich fahren.
     *
     * Die Zellen muessen dabei UNBELASTET sein - der gemessene Mittelwert wird
     * zum Nullpunkt. Schlaegt der Abgleich fehl (kein Wandler antwortet),
     * bleibt init() trotzdem erfolgreich: Der Sensor liefert dann rohe Counts
     * ohne Tara, und tared() meldet false. So ist am Boden unterscheidbar
     * "Sensor fehlt" von "Sensor da, aber nie genullt".
     *
     * @return true wenn die Pins gesetzt sind (also immer)
     */
    bool init();

    /**
     * @brief Steht bei ALLEN Wandlern der Gruppe ein neuer Messwert an?
     *
     * Der HX711 zieht DOUT auf LOW, sobald gewandelt ist. Erst wenn alle so
     * weit sind, darf getaktet werden - sonst bekaeme der noch rechnende
     * Wandler seine Impulse mitten in der Wandlung.
     */
    bool ready() const;

    /**
     * @brief Einen Satz aller Kanaele lesen - nicht blockierend.
     *
     * @param out wird nur bei Rueckgabe true beschrieben
     * @return false, wenn noch kein neuer Messwert ansteht (Normalfall im
     *         schnellen Loop) - dann einfach beim naechsten Durchlauf wieder
     *         versuchen.
     */
    bool read(ForceReading& out);

    /**
     * @brief Nullpunkt neu setzen (blockierend, bis TARE_TIMEOUT_MS).
     *
     * Blockiert bewusst: Ein Nullabgleich ueber mehrere Wandlungen laesst sich
     * nicht sinnvoll in den Loop verteilen, und er passiert nur beim Booten
     * oder auf Telecommand - nicht im Flug.
     *
     * @return true, wenn mindestens ein Sample eingesammelt wurde
     */
    bool tare(uint8_t samples = TARE_SAMPLES,
              uint32_t timeout_ms = TARE_TIMEOUT_MS);

    /** @brief Wurde je erfolgreich tariert? */
    bool tared() const { return tared_; }

    /**
     * @brief Laenger als FORCE_STALE_MS kein neuer Messwert?
     *
     * Trennt am Boden "Kraft ist konstant" von "Wandler antwortet nicht mehr" -
     * in den Messwerten selbst sieht beides gleich aus.
     */
    bool stalled() const;

    /** @brief Anzahl Kanaele dieser Gruppe. */
    uint8_t channels() const { return count_; }

    /** @brief Nullpunkt eines Kanals in rohen Counts (Diagnose am Terminal). */
    int32_t offset(uint8_t channel) const {
        return channel < count_ ? offset_[channel] : 0;
    }

private:
    uint8_t dout_[FORCE_MAX_CHANNELS] = {};
    uint8_t count_   = 0;
    uint8_t pin_sck_ = 0;
    int32_t tele_div_ = 1;      ///< siehe FORCE1_TELE_DIV / FORCE2_TELE_DIV

    bool initialized_ = false;
    bool tared_       = false;

    int32_t offset_[FORCE_MAX_CHANNELS] = {};

    uint32_t last_sample_ms_ = 0;

    /**
     * @brief Die eigentliche Taktschleife: 24 Datenbits + 1 Gain-Impuls.
     *
     * Der 25. Impuls waehlt fuer die NAECHSTE Wandlung Kanal A mit Gain 128 -
     * die Betriebsart, in der die Wiegezellen haengen.
     *
     * @param out Array mit mindestens count_ Elementen
     */
    void shiftOut24(int32_t* out);

    /// 24-Bit-Zweierkomplement auf int32 vorzeichenerweitern.
    static int32_t signExtend24(uint32_t value);

    /// Tarierten Wert auf den int16 des Downlinks bringen, mit Sat-Flag.
    static int16_t toTelemetry(int32_t counts, int32_t tele_div, bool& saturated);
};
