#include <Arduino.h>
#include "system.hpp"

// Globale System-Instanz
System* g_system = nullptr;

// ---------------------------------------------------------------------------
// Hardware-Watchdog (WDOG1 des i.MX RT1062)
// ---------------------------------------------------------------------------
// Letzte Rueckfallebene, falls loop() haengt (Endlosschleife, festgefahrener
// Treiber). Bleibt das Fuettern laenger als WATCHDOG_TIMEOUT aus, setzt der
// Chip hart zurueck. Nach dem Reset sind alle Motor-Pins hochohmig (DRV8871
// -> Coast) und das System startet in PRE_LAUNCH mit gesperrten Aktoren.
//
// Timeout = (WT + 1) * 0.5 s -> WT = 1 ergibt 1 s. Ein normaler Loop dauert
// weit unter 50 ms; einmal aktiviert, laesst sich WDOG1 nicht mehr abschalten.
static constexpr uint8_t WATCHDOG_WT = 1;   // 1 s

static void watchdogStart() {
    WDOG1_WMCR = 0;   // Power-Down-Zaehler (16-s-Reset nach Boot) aus
    WDOG1_WCR |= WDOG_WCR_SRS | WDOG_WCR_WT(WATCHDOG_WT);
    WDOG1_WCR |= WDOG_WCR_WDE | WDOG_WCR_WDT | WDOG_WCR_SRE;
}

static void watchdogFeed() {
    // Feste Fuetter-Sequenz laut Referenzhandbuch.
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
}

void setup() {
    // USB-Debug. Bewusst KEIN Warten auf den Serial-Monitor: Nach einem
    // Watchdog-Reset im Flug zaehlt jede Millisekunde bis Uplink und
    // Telemetrie wieder laufen. Der Core wartet vor setup() ohnehin 300 ms
    // auf die USB-Enumeration; wer den Boot-Banner sehen will, oeffnet den
    // Monitor vorher.
    Serial.begin(115200);

    // War der letzte Reset ein Watchdog-Reset? Am Boden zeigt ihn die
    // zurueckgesprungene Uptime im SYS/STATE-Frame; der SD-Logger braucht ihn
    // fuer seinen Haenger-Schutz (siehe sd_logger.hpp).
    const bool after_watchdog_reset = (SRC_SRSR & SRC_SRSR_WDOG_RST_B) != 0;
    SRC_SRSR = SRC_SRSR;   // Reset-Gruende quittieren (write-1-to-clear)

    // System::init() startet auch degradiert - fehlende Subsysteme meldet
    // es per Subsystem-Bits an die Bodenstation, statt anzuhalten.
    g_system = new System();
    g_system->init(after_watchdog_reset);

    watchdogStart();
}

void loop() {
    watchdogFeed();
    g_system->run();
}
