#include <Arduino.h>
#include "system.hpp"
#include "boot_guard.hpp"

// Globale System-Instanz
System* g_system = nullptr;

// ---------------------------------------------------------------------------
// Hardware-Watchdog (WDOG1 des i.MX RT1062) - siehe boot_guard.hpp
// ---------------------------------------------------------------------------
// Letzte Rueckfallebene, falls irgendetwas haengt. Bleibt das Fuettern aus,
// setzt der Chip hart zurueck. Nach dem Reset sind alle Motor-Pins
// hochohmig (DRV8871 -> Coast) und das System startet mit gesperrten Aktoren.
//
// Startet als ALLERERSTES (Timeout WATCHDOG_BOOT_S), damit auch ein Haenger
// in der Initialisierung zum Reset fuehrt - und der Boot-Guard danach das
// schuldige Subsystem auslaesst. Nach init() gilt WATCHDOG_RUN_S (1 s).

void setup() {
    // War der letzte Reset ein Watchdog-Reset? Fuer den Boot-Guard
    // (Brotkrumen, Reset-Zaehler) und die Diagnose am Boden (SYS/BOOT).
    const bool after_watchdog_reset = (SRC_SRSR & SRC_SRSR_WDOG_RST_B) != 0;
    SRC_SRSR = SRC_SRSR;   // Reset-Gruende quittieren (write-1-to-clear)

    bootguard::watchdogStart();
    bootguard::begin(after_watchdog_reset);

    // USB-Debug. Bewusst KEIN Warten auf den Serial-Monitor: Nach einem
    // Watchdog-Reset im Flug zaehlt jede Millisekunde bis Uplink und
    // Telemetrie wieder laufen. Der Core wartet vor setup() ohnehin 300 ms
    // auf die USB-Enumeration; wer den Boot-Banner sehen will, oeffnet den
    // Monitor vorher.
    Serial.begin(115200);

    // System::init() startet auch degradiert - fehlende Subsysteme meldet
    // es per Subsystem-Bits an die Bodenstation, statt anzuhalten.
    g_system = new System();
    g_system->init(after_watchdog_reset);

    bootguard::watchdogTimeout(bootguard::WATCHDOG_RUN_S);
}

void loop() {
    bootguard::watchdogFeed();
    g_system->run();
}
