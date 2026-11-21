#include "boot_guard.hpp"
#include <Arduino.h>

// SNVS_LPGPR0: Brotkrume  = STAGE_MAGIC << 8 | Stage
// SNVS_LPGPR1: Zaehler    = COUNT_MAGIC << 8 | Watchdog-Resets in Folge
// Die Magie unterscheidet einen gueltigen Wert von Zufallsinhalt nach dem
// ersten Einschalten.
static constexpr uint32_t STAGE_MAGIC = 0x4D4753u;   // "MGS"
static constexpr uint32_t COUNT_MAGIC = 0x4D4743u;   // "MGC"

static Stage    s_current   = Stage::NONE;
static Stage    s_culprit   = Stage::NONE;
static uint8_t  s_resets    = 0;
static bool     s_after_wd  = false;
static bool     s_minimal   = false;
static uint8_t  s_disabled  = 0;
static bool     s_counter_cleared = false;

namespace bootguard {

void watchdogStart() {
    WDOG1_WMCR = 0;   // Power-Down-Zaehler (16-s-Reset nach Boot) aus
    // Timeout = (WT + 1) * 0,5 s
    WDOG1_WCR |= WDOG_WCR_SRS | WDOG_WCR_WT(WATCHDOG_BOOT_S * 2 - 1);
    WDOG1_WCR |= WDOG_WCR_WDE | WDOG_WCR_WDT | WDOG_WCR_SRE;
    watchdogFeed();
}

void watchdogTimeout(uint8_t seconds) {
    // WT darf laut Referenzhandbuch auch bei laufendem Watchdog geaendert
    // werden (Linux imx2_wdt macht das genauso); gilt ab dem naechsten Fuettern.
    const uint16_t wcr = WDOG1_WCR;
    WDOG1_WCR = static_cast<uint16_t>((wcr & ~WDOG_WCR_WT(0xFF)) | WDOG_WCR_WT(seconds * 2 - 1));
    watchdogFeed();
}

void watchdogFeed() {
    // Feste Fuetter-Sequenz laut Referenzhandbuch.
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
}

void begin(bool after_watchdog_reset) {
    s_after_wd = after_watchdog_reset;

    const uint32_t gp0 = SNVS_LPGPR0;
    const uint32_t gp1 = SNVS_LPGPR1;
    const uint8_t last_stage = (gp0 >> 8) == STAGE_MAGIC ? static_cast<uint8_t>(gp0 & 0xFF) : 0;
    const uint8_t count      = (gp1 >> 8) == COUNT_MAGIC ? static_cast<uint8_t>(gp1 & 0xFF) : 0;

    if (after_watchdog_reset) {
        s_culprit = static_cast<Stage>(last_stage);
        s_resets  = count < 0xFF ? count + 1 : 0xFF;
    } else {
        s_culprit = Stage::NONE;
        s_resets  = 0;
    }
    SNVS_LPGPR1 = (COUNT_MAGIC << 8) | s_resets;

    // Schuldiges Subsystem fuer diesen Start aus.
    switch (s_culprit) {
        case Stage::INIT_SD: case Stage::LOOP_SD: case Stage::LOOP_SD_FORMAT:
            s_disabled |= GUARD_OFF_SD; break;
        case Stage::INIT_IMU: case Stage::LOOP_IMU:
            s_disabled |= GUARD_OFF_IMU; break;
        case Stage::INIT_FORCE: case Stage::LOOP_FORCE:
            s_disabled |= GUARD_OFF_FORCE; break;
        case Stage::INIT_CAM: case Stage::LOOP_CAM:
            s_disabled |= GUARD_OFF_CAM; break;
        default: break;   // unverzichtbar (Uplink, Zustand, Motoren, ARM - siehe .hpp)
    }

    s_minimal = s_resets >= MINIMAL_MODE_RESETS;
    if (s_minimal) s_disabled |= GUARD_OFF_SD | GUARD_OFF_CAM | GUARD_OFF_FORCE;

    stage(Stage::NONE);
}

void stage(Stage s) {
    s_current = s;
    SNVS_LPGPR0 = (STAGE_MAGIC << 8) | static_cast<uint8_t>(s);
}

Stage currentStage() { return s_current; }
Stage culprit() { return s_culprit; }
uint8_t resetsInRow() { return s_resets; }
bool afterWatchdogReset() { return s_after_wd; }
bool minimalMode() { return s_minimal; }
uint8_t disabledMask() { return s_disabled; }

void loopTick(uint32_t now_ms) {
    if (s_counter_cleared || now_ms < STABLE_AFTER_MS) return;
    s_counter_cleared = true;
    SNVS_LPGPR1 = (COUNT_MAGIC << 8) | 0;
}

const char* stageName(Stage s) {
    switch (s) {
        case Stage::NONE:            return "-";
        case Stage::INIT_PERSIST:    return "INIT_PERSIST";
        case Stage::INIT_IMU:        return "INIT_IMU";
        case Stage::INIT_DOWNLINK:   return "INIT_DOWNLINK";
        case Stage::INIT_SD:         return "INIT_SD";
        case Stage::INIT_MOTOR:      return "INIT_MOTOR";
        case Stage::INIT_FORCE:      return "INIT_FORCE";
        case Stage::INIT_ARM:        return "INIT_ARM";
        case Stage::INIT_CAM:        return "INIT_CAM";
        case Stage::INIT_REXUS:      return "INIT_REXUS";
        case Stage::INIT_STATE:      return "INIT_STATE";
        case Stage::LOOP_UPLINK:     return "LOOP_UPLINK";
        case Stage::LOOP_ARM:        return "LOOP_ARM";
        case Stage::LOOP_CAM:        return "LOOP_CAM";
        case Stage::LOOP_STATE:      return "LOOP_STATE";
        case Stage::LOOP_FLIGHT:     return "LOOP_FLIGHT";
        case Stage::LOOP_MOTOR:      return "LOOP_MOTOR";
        case Stage::LOOP_FORCE:      return "LOOP_FORCE";
        case Stage::LOOP_IMU:        return "LOOP_IMU";
        case Stage::LOOP_TELEMETRY:  return "LOOP_TELEMETRY";
        case Stage::LOOP_CHECKPOINT: return "LOOP_CHECKPOINT";
        case Stage::LOOP_SD:         return "LOOP_SD";
        case Stage::LOOP_SD_FORMAT:  return "LOOP_SD_FORMAT";
    }
    return "?";
}

}  // namespace bootguard
