#pragma once

#include <cstdint>

/**
 * @file boot_guard.hpp
 * @brief Watchdog, "Wo war der OBC?" und Schutz vor Reset-Schleifen
 *
 * Der OBC darf NIRGENDS haengen bleiben und nach einem Haenger nicht in eine
 * Reset-Schleife geraten. Drei Bausteine:
 *
 * 1. Hardware-Watchdog (WDOG1). Startet als ALLERERSTES in setup() mit
 *    WATCHDOG_BOOT_S - so faengt er auch einen Haenger in der Initialisierung
 *    (frueher lief er erst nach init()). Nach init() gilt WATCHDOG_RUN_S.
 *
 * 2. Brotkrumen (Stage): Vor jedem Abschnitt, der haengen koennte, steht in
 *    SNVS_LPGPR0, wo der OBC gerade ist. Das Register liegt in der
 *    SNVS-Domaene und uebersteht den Watchdog-Reset. Nach einem Watchdog-Reset
 *    steht darin also der Schuldige. Ist es ein verzichtbares Subsystem
 *    (SD-Karte, IMU, Kraftsensoren, Kameras), bleibt genau dieses fuer DIESEN
 *    Start aus - statt beim naechsten Mal wieder dort zu haengen.
 *    Der ARM wird NIE abgeschaltet (Nutzer, 19.11.2026: "der Watchdog soll
 *    den Arm unter keinen Umstaenden unnoetig stoppen") - auch nicht, wenn
 *    der Haenger in der Arm-UART lag.
 *
 * 3. Zaehler (SNVS_LPGPR1): Watchdog-Resets in Folge. Ab
 *    MINIMAL_MODE_RESETS startet der OBC im Minimalmodus: alle verzichtbaren
 *    Subsysteme (SD, Kameras, Kraftsensoren) aus. Zustandsmaschine, Uplink,
 *    Downlink, Motoren (HDRM), IMU und Arm laufen weiter. Laeuft der OBC
 *    STABLE_AFTER_MS ohne Reset, wird der Zaehler geloescht.
 *
 * Ein Stromausfall loescht beide Register (ohne Knopfzelle an VBAT) - das
 * ist gewollt: Nach einem Stromzyklus startet alles normal.
 */

/// Wo der OBC gerade ist. Werte gehen per SYS/BOOT an den Boden - FEST.
enum class Stage : uint8_t {
    NONE = 0,
    // Initialisierung
    INIT_PERSIST = 1, INIT_IMU = 2, INIT_DOWNLINK = 3, INIT_SD = 4, INIT_MOTOR = 5,
    INIT_FORCE = 6, INIT_ARM = 7, INIT_CAM = 8, INIT_REXUS = 9, INIT_STATE = 10,
    // Loop
    LOOP_UPLINK = 20, LOOP_ARM = 21, LOOP_CAM = 22, LOOP_STATE = 23, LOOP_FLIGHT = 24,
    LOOP_MOTOR = 25, LOOP_FORCE = 26, LOOP_IMU = 27, LOOP_TELEMETRY = 28,
    LOOP_CHECKPOINT = 29, LOOP_SD = 30, LOOP_SD_FORMAT = 31,
};

/// Subsysteme, die nach einem Haenger fuer einen Start aus bleiben koennen.
/// Bitmaske - geht per SYS/BOOT an den Boden.
static constexpr uint8_t GUARD_OFF_SD    = 0x01;
static constexpr uint8_t GUARD_OFF_IMU   = 0x02;
static constexpr uint8_t GUARD_OFF_FORCE = 0x04;
static constexpr uint8_t GUARD_OFF_CAM   = 0x08;
static constexpr uint8_t GUARD_OFF_ARM   = 0x10;   ///< reserviert - wird NIE gesetzt (siehe oben)

namespace bootguard {

static constexpr uint8_t  WATCHDOG_BOOT_S     = 5;      ///< Timeout waehrend init()
static constexpr uint8_t  WATCHDOG_RUN_S      = 1;      ///< Timeout im Loop
static constexpr uint8_t  MINIMAL_MODE_RESETS = 3;      ///< ab so vielen Resets in Folge
static constexpr uint32_t STABLE_AFTER_MS     = 60000;  ///< danach zaehlt der Zaehler neu

/// Watchdog mit WATCHDOG_BOOT_S starten. Ganz am Anfang von setup().
void watchdogStart();
/// Timeout umstellen (wirkt ab dem naechsten Fuettern).
void watchdogTimeout(uint8_t seconds);
void watchdogFeed();

/**
 * @brief Register auswerten. Einmal, direkt nach watchdogStart().
 * @param after_watchdog_reset true, wenn der letzte Reset vom Watchdog kam
 */
void begin(bool after_watchdog_reset);

/// Brotkrume setzen. Kostet einen Registerzugriff.
void stage(Stage s);
Stage currentStage();

/// Stage, in der der letzte Watchdog-Reset zuschlug (NONE = kein Watchdog-Reset).
Stage culprit();
/// Watchdog-Resets in Folge (inkl. des letzten).
uint8_t resetsInRow();
bool afterWatchdogReset();
/// Minimalmodus aktiv?
bool minimalMode();
/// Diese Subsysteme bleiben fuer diesen Start aus (GUARD_OFF_*).
uint8_t disabledMask();
inline bool disabled(uint8_t guard_bit) { return (disabledMask() & guard_bit) != 0; }

/// Jeden Loop: setzt den Zaehler nach STABLE_AFTER_MS zurueck.
void loopTick(uint32_t now_ms);

/// Klartext fuer Log und SD-Karte.
const char* stageName(Stage s);

/// Brotkrume fuer einen Block setzen und danach die alte wiederherstellen.
class Scope {
public:
    explicit Scope(Stage s) : prev_(currentStage()) { stage(s); }
    ~Scope() { stage(prev_); }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
private:
    Stage prev_;
};

}  // namespace bootguard
