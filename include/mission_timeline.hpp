#pragma once

#include <cstdint>

/**
 * @file mission_timeline.hpp
 * @brief Zeitplan der REXUS-Flugsequenz - ALLE Flugzeiten an einer Stelle
 *
 *   PRE_LAUNCH ──SODS──► ARMED ──LO──► ASCENT ──SOE oder 70 s──► EXPERIMENT (t0)
 *
 *   EXPERIMENT   t0 + 2,000 s  HDRM 1 oeffnen
 *                t0 + 2,010 s  HDRM 2 oeffnen
 *                jedes HDRM wird wiederholt (Wackeln + neuer Versuch), bis der
 *                Encoder "offen" meldet; beide nachweislich offen ──► ROBOT_ARM
 *   ROBOT_ARM    Roboterarm (Platzhalter) bis t0 + 115 s ──► DESCENT
 *   DESCENT      HDRM 1 schliessen, 10 ms spaeter HDRM 2 - ebenfalls wiederholt,
 *                bis zu; dann den Motor blockieren (alle Motor-Pins HIGH) bis
 *                Strom aus / STATE_RESET
 *
 * Zeitbezug: In ASCENT laeuft die Zeit ab LO, ab EXPERIMENT ab t0 (Eintritt in
 * EXPERIMENT). ROBOT_ARM und DESCENT behalten t0.
 *
 * Neustart im Flug: Die verstrichene Zeit wird jede volle Sekunde im EEPROM
 * gesichert und nach einem Neustart fortgesetzt (siehe System::init). Die
 * Sequenz laeuft danach also hoechstens ~1 s plus Ausfalldauer verspaetet weiter.
 *
 * Gegenstueck am Boden: Die HDRM-Winkel entsprechen den Knoepfen
 * "HDRM Open/Close" im Testmodus der GUI (HALF_TURN_DEG in telecommand.ts).
 */
namespace timeline {

/// Laengste Wartezeit in ASCENT ab LO. Kommt SOE vorher, startet EXPERIMENT sofort.
constexpr uint32_t ASCENT_MAX_MS = 70000;

/// HDRM 1 oeffnet zu diesem Zeitpunkt ab t0.
constexpr uint32_t HDRM_OPEN_AT_MS = 2000;

/// Versatz zwischen HDRM 1 und HDRM 2 - beim Oeffnen wie beim Schliessen.
/// Verhindert, dass beide Motoren gleichzeitig anlaufen (Anlaufstrom).
constexpr uint32_t HDRM_STAGGER_MS = 10;

/// Ende des Experiments ab t0: Wechsel nach DESCENT, HDRMs schliessen.
constexpr uint32_t DESCENT_AT_MS = 115000;

/// Ende des Mikrogravitationsfensters ab t0. Nur Referenz - die HDRMs muessen
/// davor zu sein, deshalb DESCENT_AT_MS mit Abstand davor.
constexpr uint32_t MICROGRAVITY_END_MS = 120000;

/// Relative Drehwinkel der HDRM-Fahrten (MOTOR_TURN, wie im Testmodus).
constexpr int16_t HDRM_OPEN_DEG  = -165;
constexpr int16_t HDRM_CLOSE_DEG = +165;

/// Ab so viel Grad in Oeffnungsrichtung gilt ein HDRM als OFFEN - auch wenn es
/// vor den vollen HDRM_OPEN_DEG stehen bleibt (Nutzer, 19.11.2026: "wegen 10
/// Grad soll der Arm nicht blockiert sein"). Darunter: wackeln + neuer Versuch.
constexpr int16_t HDRM_OPEN_MIN_DEG = 140;
/// So nah an der Ausgangslage gilt ein HDRM beim Schliessen als ZU.
constexpr int16_t HDRM_CLOSE_TOL_DEG = 25;

static_assert(HDRM_OPEN_AT_MS + HDRM_STAGGER_MS < DESCENT_AT_MS,
              "HDRMs muessen vor DESCENT geoeffnet sein");
static_assert(HDRM_OPEN_MIN_DEG > 0 && HDRM_OPEN_MIN_DEG <= -HDRM_OPEN_DEG,
              "HDRM_OPEN_MIN_DEG muss zwischen 0 und dem vollen Oeffnungswinkel liegen");
static_assert(DESCENT_AT_MS < MICROGRAVITY_END_MS,
              "HDRMs muessen vor Ende der Mikrogravitation schliessen");

}  // namespace timeline
