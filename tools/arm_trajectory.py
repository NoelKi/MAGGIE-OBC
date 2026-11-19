#!/usr/bin/env python3
"""
Trajektorientest an bis zu ZWEI Gelenken des Roboterarms (Bodentest).

Die ROS-Simulation liefert eine CSV: eine Zeile pro Zeitschritt (0,1 s), eine
Spalte pro Gelenk (joint 1, joint 2, ...), Winkel in rad. Dieses Skript

  1. wandelt 1-2 Spalten in die Tabelle include/arm_test_trajectory.hpp um, die
     der OBC an Bord synchron abspielt (ArmTrajPlayer, nur im TEST-Zustand), und
  2. startet bzw. stoppt die Fahrt über den MAGGIE-Server.

Warum an Bord und nicht vom Boden gestreamt: Der Uplink kennt nur relative
Jog-Schritte in ganzen Grad, und die Laufzeit über das RXSM schwankt. Der OBC
schickt die Ziele dagegen exakt alle 100 ms als absolutes MOVE_ABS.

Die Tabelle enthält je Gelenk Winkel RELATIV zum ersten Punkt. Der OBC addiert
sie zum Winkel, den das Gelenk beim Start meldet - die Fahrt läuft also um den
aktuellen Stand herum, nicht auf die absoluten Winkel der Simulation.

Grenzen (prüfen Skript UND Firmware), je Gelenk:
  - jeder Punkt innerhalb ±30° um den Start
  - Endpunkt = Startpunkt (fehlt das, hängt das Skript eine sanfte Rückfahrt an)
  - höchstens 10 °/s und 4 °/s² (sonst mit --slowdown zeitlich strecken)
  - höchstens 2 Gelenke (Freigabe des Teams, CLAUDE.md Regel 2)

Beispiele
---------
  # Beispiel-CSV erzeugen (3 Gelenke, Gelenk 1 und 2 bewegen sich)
  python3 tools/arm_trajectory.py example -o traj_example.csv

  # Spalten 1 und 2 in die Firmware-Tabelle umwandeln (+ Plot)
  python3 tools/arm_trajectory.py convert traj_example.csv --joints 1 2 --plot traj.png

  # danach: Firmware bauen und flashen (pio run -t upload)

  # In der GUI TEST betreten, dann Spalte 1 -> Node 5, Spalte 2 -> Node 6
  python3 tools/arm_trajectory.py start --nodes 5 6
  python3 tools/arm_trajectory.py stop  --nodes 5 6

Flug-Trajektorie (alle Gelenke, ABSOLUTE Winkel, Zustand ROBOT_ARM)
-------------------------------------------------------------------
  # Spalten 1..6 der Flug-CSV auf Node 1..6, schreibt include/arm_flight_trajectory.hpp
  python3 tools/arm_trajectory.py flight flug.csv --joints 1 2 3 4 5 6 --nodes 1 2 3 4 5 6 --plot flug.png

  # eingebaute Tabelle anzeigen und ihre CRC prüfen
  python3 tools/arm_trajectory.py flight-info

  Prüft: Ende = Start (Verstaupose, keine automatische Rückfahrt), höchstens
  10 °/s und 4 °/s², höchstens 90 s. Danach Firmware bauen und flashen; die
  GUI zeigt unter "Flug-Trajektorie" Tabelle OK, Gelenkzahl und CRC-Endbyte.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path

# ── Muss zu ArmTrajPlayer (include/arm_traj_player.hpp) passen ──────────────
DT_S = 0.1                 # Takt der ROS-CSV und des Players
MAX_JOINTS = 2             # höchstens so viele Gelenke gleichzeitig
MAX_DEG = 30.0             # größte Auslenkung um den Start
MAX_VEL_DEG_S = 10.0       # höchste Geschwindigkeit (Vorgabe Team)
ACCEL_DEG_S2 = 4.0         # Beschleunigung des Players = höchste der Bahn (Vorgabe Team)
ACCEL_WINDOW = 5           # Beschleunigung über 5 Schritte (0,5 s) gemittelt prüfen
RETURN_VEL_DEG_S = 5.0     # Spitzengeschwindigkeit der angehängten Rückfahrt
RETURN_MIN_S = 2.0         # Rückfahrt mindestens so lang (hält sie unter ACCEL)
MAX_POINTS = 6000          # 10 min - Tabelle liegt im Flash

URAD_PER_DEG = math.pi / 180.0 * 1e6

# ── Flug-Trajektorie: muss zu ArmFlightPlayer (include/arm_flight_player.hpp) passen
FLIGHT_MAX_JOINTS = 8          # ARM_MAX_JOINTS (arm_hal.hpp)
FLIGHT_MAX_VEL_DEG_S = 10.0    # ArmFlightPlayer::MAX_VEL_DEG_S
FLIGHT_ACCEL_DEG_S2 = 4.0      # ArmFlightPlayer::ACCEL_DEG_S2
FLIGHT_MAX_DURATION_S = 90.0   # ArmFlightPlayer::MAX_DURATION_MS
FLIGHT_END_TOL_DEG = 0.1       # Ende = Start (Verstaupose), sonst Fehler

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_HEADER = ROOT / "include" / "arm_test_trajectory.hpp"
FLIGHT_HEADER = ROOT / "include" / "arm_flight_trajectory.hpp"
DEFAULT_SERVER = "http://localhost:3000"


def fail(msg: str) -> None:
    print(f"FEHLER: {msg}", file=sys.stderr)
    sys.exit(1)


# ── CSV lesen ────────────────────────────────────────────────────────────────

def read_columns(path: Path, joints: list[int], unit: str) -> list[list[float]]:
    """Spalten `joints` (1-basiert) in Grad. Kopfzeilen werden übersprungen."""
    cols: list[list[float]] = [[] for _ in joints]
    with path.open(newline="") as f:
        sample = f.read(4096)
        f.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t ")
        except csv.Error:
            dialect = csv.excel
        for lineno, row in enumerate(csv.reader(f, dialect), start=1):
            row = [c.strip() for c in row if c.strip() != ""]
            if not row:
                continue
            try:
                nums = [float(c) for c in row]
            except ValueError:
                if cols[0]:
                    fail(f"Zeile {lineno}: keine Zahl ({row}) - mitten in den Daten.")
                continue   # Kopfzeile
            for k, j in enumerate(joints):
                if j > len(nums):
                    fail(f"Zeile {lineno} hat nur {len(nums)} Spalten, Gelenk {j} gibt es nicht.")
                cols[k].append(nums[j - 1])
    if len(cols[0]) < 2:
        fail("Weniger als 2 Datenzeilen in der CSV.")
    if unit == "rad":
        cols = [[math.degrees(v) for v in c] for c in cols]
    return cols


# ── Aufbereiten und prüfen ───────────────────────────────────────────────────

def stretch(points: list[float], factor: float) -> list[float]:
    """Zeitlich strecken (factor > 1 = langsamer), linear interpoliert."""
    if factor == 1.0:
        return points
    n_new = int(round((len(points) - 1) * factor)) + 1
    out = []
    for i in range(n_new):
        x = i / factor
        k = min(int(x), len(points) - 2)
        frac = x - k
        out.append(points[k] + (points[k + 1] - points[k]) * frac)
    return out


def append_return(cols: list[list[float]]) -> int:
    """Sanfte Rückfahrt (Kosinus) aller Spalten vom letzten Punkt auf 0 -
    gemeinsam, gleich lang, ausgelegt für die Spalte mit dem weitesten Weg."""
    far = max(abs(c[-1]) for c in cols)
    if far < 0.01:
        return 0
    # Kosinus über T: Spitze v = far·π/(2T), Spitze a = far·π²/(2T²) - beide begrenzen.
    duration = max(RETURN_MIN_S,
                   far * math.pi / (2.0 * RETURN_VEL_DEG_S),
                   math.pi * math.sqrt(far / (2.0 * ACCEL_DEG_S2)))
    n = math.ceil(duration / DT_S)
    for c in cols:
        last = c[-1]
        c.extend(last * (1.0 + math.cos(math.pi * i / n)) / 2.0 for i in range(1, n + 1))
    return n


def stats(offsets: list[float]) -> dict:
    vel = [(b - a) / DT_S for a, b in zip(offsets, offsets[1:])]
    # Über ACCEL_WINDOW Schritte: Die Differenz zweier 0,1-s-Geschwindigkeiten
    # rauscht stark und ist nicht das, was das Gelenk fahren muss.
    w = ACCEL_WINDOW
    acc = [(vel[i + w] - vel[i]) / (w * DT_S) for i in range(len(vel) - w)]
    return {
        "min": min(offsets),
        "max": max(offsets),
        "vmax": max((abs(v) for v in vel), default=0.0),
        "amax": max((abs(a) for a in acc), default=0.0),
        "vel": vel,
    }


# ── Header schreiben ─────────────────────────────────────────────────────────

def write_header(path: Path, urad: list[list[int]], source: str, joints: list[int], note: str) -> None:
    n, m = len(urad[0]), len(urad)
    rows = []
    for i in range(n):
        rows.append("    {" + ", ".join(f"{urad[k][i]:9d}" for k in range(m)) + "},")
    body = "\n".join(rows)
    cols = ", ".join(str(j) for j in joints)
    path.write_text(f"""#pragma once
#include <Arduino.h>

// ERZEUGT von tools/arm_trajectory.py - nicht von Hand aendern.
// Quelle: {source}, CSV-Spalte(n) {cols}{note}
// {n} Punkte a {int(DT_S * 1000)} ms = {n * DT_S:.1f} s, {m} Gelenk(e).
// Winkel in urad RELATIV zum Startwinkel des jeweiligen Gelenks; erster und
// letzter Punkt sind 0 (Ende = Start). Spalte k faehrt die k-te Node aus
// ARM_TRAJ_TEST. Abgespielt von ArmTrajPlayer.

static constexpr uint32_t ARM_TRAJ_DT_MS  = {int(DT_S * 1000)};
static constexpr uint8_t  ARM_TRAJ_JOINTS = {m};
static constexpr uint16_t ARM_TRAJ_LEN    = {n};

static const int32_t ARM_TRAJ_OFFSET_URAD[ARM_TRAJ_LEN][ARM_TRAJ_JOINTS] PROGMEM = {{
{body}
}};
""")


def header_joint_count(path: Path) -> int | None:
    try:
        m = re.search(r"ARM_TRAJ_JOINTS\s*=\s*(\d+)", path.read_text())
    except OSError:
        return None
    return int(m.group(1)) if m else None


def plot(path: Path, cols: list[list[float]], joints: list[int], n_return: int,
         flight: bool = False) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Hinweis: matplotlib fehlt - kein Plot.")
        return
    t = [i * DT_S for i in range(len(cols[0]))]
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 5))
    for c, j in zip(cols, joints):
        a1.plot(t, c, lw=1.5, label=f"CSV-Spalte {j}")
        a2.plot(t[1:], stats(c)["vel"], lw=1.2)
    if not flight:
        for y in (MAX_DEG, -MAX_DEG):
            a1.axhline(y, color="r", ls="--", lw=0.8)
    vlim = FLIGHT_MAX_VEL_DEG_S if flight else MAX_VEL_DEG_S
    for y in (vlim, -vlim):
        a2.axhline(y, color="r", ls="--", lw=0.8)
    if n_return:
        a1.axvspan(t[-n_return - 1], t[-1], color="0.9", label="angehängte Rückfahrt")
    a1.legend(loc="upper right")
    a1.set_ylabel("Gelenkwinkel absolut [°]" if flight else "Winkel rel. zum Start [°]")
    a2.set_ylabel("Geschwindigkeit [°/s]")
    a2.set_xlabel("Zeit [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"Plot: {path}")


# ── Befehle ──────────────────────────────────────────────────────────────────

def cmd_convert(a: argparse.Namespace) -> None:
    if not 1 <= len(a.joints) <= MAX_JOINTS:
        fail(f"1 bis {MAX_JOINTS} Spalten angeben (--joints), nicht {len(a.joints)}.")
    if len(set(a.joints)) != len(a.joints):
        fail("Spalte doppelt angegeben.")

    cols = read_columns(Path(a.csv), a.joints, a.unit)
    cols = [stretch(c, a.slowdown) for c in cols]
    cols = [[v - c[0] for v in c] for c in cols]

    for c, j in zip(cols, a.joints):
        lo, hi = min(c), max(c)
        if lo < -MAX_DEG or hi > MAX_DEG:
            fail(f"Spalte {j} reicht von {lo:+.2f}° bis {hi:+.2f}° um den Start - "
                 f"erlaubt sind ±{MAX_DEG:.0f}°. Bitte in ROS kleiner planen.")

    n_return = append_return(cols)
    for c in cols:
        c[0] = 0.0
        c[-1] = 0.0
    if len(cols[0]) > MAX_POINTS:
        fail(f"{len(cols[0])} Punkte - mehr als {MAX_POINTS} ({MAX_POINTS * DT_S / 60:.0f} min).")

    st = [stats(c) for c in cols]
    vmax = max(s["vmax"] for s in st)
    amax = max(s["amax"] for s in st)
    if vmax > MAX_VEL_DEG_S or amax > ACCEL_DEG_S2:
        # Strecken um f teilt Geschwindigkeit durch f, Beschleunigung durch f².
        need = a.slowdown * max(vmax / MAX_VEL_DEG_S, math.sqrt(amax / ACCEL_DEG_S2), 1.0)
        fail(f"Bahn zu schnell: max. {vmax:.1f}°/s (erlaubt {MAX_VEL_DEG_S:.0f}), "
             f"max. {amax:.1f}°/s² (erlaubt {ACCEL_DEG_S2:.0f}). "
             f"Mit --slowdown {math.ceil(need * 10) / 10:.1f} (oder mehr) zeitlich strecken.")

    urad = [[int(round(v * URAD_PER_DEG)) for v in c] for c in cols]
    note = f", zeitlich gestreckt x{a.slowdown:g}" if a.slowdown != 1.0 else ""
    write_header(Path(a.output), urad, Path(a.csv).name, a.joints, note)

    print(f"Tabelle: {a.output}")
    print(f"  {len(cols[0])} Punkte = {len(cols[0]) * DT_S:.1f} s, {len(cols)} Gelenk(e)")
    for s, j in zip(st, a.joints):
        print(f"  Spalte {j}: {s['min']:+.2f}° .. {s['max']:+.2f}° um den Start, "
              f"max. {s['vmax']:.1f}°/s, max. {s['amax']:.1f}°/s²")
    if n_return:
        print(f"  Ende != Start: Rückfahrt mit {n_return} Punkten ({n_return * DT_S:.1f} s) angehängt.")
    print("Nächster Schritt: Firmware bauen und flashen (pio run -t upload).")
    print(f"Starten dann mit {len(cols)} Node(s): start --nodes " +
          " ".join(f"<Node für Spalte {j}>" for j in a.joints))

    if a.plot:
        plot(Path(a.plot), cols, a.joints, n_return)


# ── Flug-Trajektorie ─────────────────────────────────────────────────────────

def crc32_mpeg2(data: bytes) -> int:
    """CRC-32/MPEG-2 wie ArmLink::crc32 (Pruefwert "123456789" -> 0x0376E6E7)."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else (crc << 1)
            crc &= 0xFFFFFFFF
    return crc


def table_crc(urad: list[list[int]]) -> int:
    """CRC über die Tabelle, zeilenweise, int32 little-endian - genau das
    Speicherabbild von ARM_FLIGHT_URAD auf dem Teensy."""
    n = len(urad[0])
    raw = b"".join(urad[k][i].to_bytes(4, "little", signed=True)
                   for i in range(n) for k in range(len(urad)))
    return crc32_mpeg2(raw)


def write_flight_header(path: Path, urad: list[list[int]], nodes: list[int],
                        source: str, joints: list[int], note: str) -> int:
    n, m = len(urad[0]), len(urad)
    crc = table_crc(urad)
    rows = "\n".join("    {" + ", ".join(f"{urad[k][i]:11d}" for k in range(m)) + "},"
                     for i in range(n))
    path.write_text(f"""#pragma once
#include <Arduino.h>

// ERZEUGT von tools/arm_trajectory.py flight - nicht von Hand aendern.
// Quelle: {source}, CSV-Spalte(n) {", ".join(map(str, joints))}{note}
// {n} Punkte a {int(DT_S * 1000)} ms = {(n - 1) * DT_S:.1f} s, {m} Gelenk(e).
// ABSOLUTE Gelenkwinkel in urad. Erster Punkt = Startpose = letzter Punkt
// (Verstaupose). Spalte k faehrt Node ARM_FLIGHT_NODES[k]. Abgespielt von
// ArmFlightPlayer im Zustand ROBOT_ARM. ARM_FLIGHT_CRC32: CRC-32/MPEG-2 ueber
// ARM_FLIGHT_URAD (int32 little-endian, zeilenweise) - der OBC prueft sie
// beim Booten und faehrt bei Abweichung nicht.

static constexpr bool     ARM_FLIGHT_VALID  = true;
static constexpr uint32_t ARM_FLIGHT_DT_MS  = {int(DT_S * 1000)};
static constexpr uint8_t  ARM_FLIGHT_JOINTS = {m};
static constexpr uint16_t ARM_FLIGHT_LEN    = {n};
static constexpr uint32_t ARM_FLIGHT_CRC32  = 0x{crc:08X}u;
static constexpr uint8_t  ARM_FLIGHT_NODES[ARM_FLIGHT_JOINTS] = {{{", ".join(map(str, nodes))}}};

static const int32_t ARM_FLIGHT_URAD[ARM_FLIGHT_LEN][ARM_FLIGHT_JOINTS] PROGMEM = {{
{rows}
}};
""")
    return crc


def read_flight_header(path: Path) -> dict:
    """Liest eine erzeugte Flug-Tabelle zurück (für flight-info)."""
    text = path.read_text()

    def const(name: str) -> str:
        m = re.search(rf"{name}\s*=\s*([^;]+);", text)
        if not m:
            fail(f"{name} fehlt in {path}.")
        return m.group(1).strip()

    valid = const("ARM_FLIGHT_VALID") == "true"
    joints = int(const("ARM_FLIGHT_JOINTS"))
    n = int(const("ARM_FLIGHT_LEN"))
    crc = int(const("ARM_FLIGHT_CRC32").rstrip("uU"), 16)
    nodes = [int(v) for v in re.search(r"ARM_FLIGHT_NODES\[[^\]]*\]\s*=\s*\{([^}]*)\}", text)
             .group(1).split(",")]
    body = text.split("PROGMEM = {", 1)[1]
    rows = re.findall(r"\{([^{}]*)\}", body)
    table = [[int(v) for v in r.split(",")] for r in rows]
    if len(table) != n or any(len(r) != joints for r in table):
        fail(f"{path}: Tabelle hat {len(table)} Zeilen statt {n} oder falsche Spaltenzahl.")
    urad = [[table[i][k] for i in range(n)] for k in range(joints)]
    return {"valid": valid, "nodes": nodes, "crc": crc, "urad": urad}


def cmd_flight(a: argparse.Namespace) -> None:
    joints, nodes = a.joints, a.nodes
    if not 1 <= len(joints) <= FLIGHT_MAX_JOINTS:
        fail(f"1 bis {FLIGHT_MAX_JOINTS} Spalten angeben (--joints), nicht {len(joints)}.")
    if len(set(joints)) != len(joints):
        fail("Spalte doppelt angegeben.")
    if len(nodes) != len(joints):
        fail(f"{len(joints)} Spalten, aber {len(nodes)} Nodes - je Spalte genau eine Node (--nodes).")
    if len(set(nodes)) != len(nodes) or any(not 1 <= nd <= FLIGHT_MAX_JOINTS for nd in nodes):
        fail(f"Nodes müssen eindeutig und 1..{FLIGHT_MAX_JOINTS} sein.")

    cols = read_columns(Path(a.csv), joints, a.unit)
    cols = [stretch(c, a.slowdown) for c in cols]

    # Ende = Start: Der Arm muss in der Verstaupose enden, in der er startet.
    # KEINE automatische Rückfahrt - ein direkter Weg kann kollidieren.
    for c, j in zip(cols, joints):
        if abs(c[-1] - c[0]) > FLIGHT_END_TOL_DEG:
            fail(f"Spalte {j} endet bei {c[-1]:+.3f}°, startet bei {c[0]:+.3f}° - die Bahn muss "
                 f"in der Startpose (Verstaupose) enden (Toleranz {FLIGHT_END_TOL_DEG}°). "
                 "Bitte in der Planung korrigieren - eine automatische Rückfahrt gibt es "
                 "im Flug bewusst nicht (Kollisionsgefahr).")
        c[-1] = c[0]

    duration = (len(cols[0]) - 1) * DT_S
    if duration > FLIGHT_MAX_DURATION_S:
        fail(f"Bahn dauert {duration:.1f} s - höchstens {FLIGHT_MAX_DURATION_S:.0f} s, damit der "
             "Arm vor DESCENT (t0 + 115 s) mit Reserve für Neustarts verstaut ist.")

    st = [stats(c) for c in cols]
    vmax = max(s["vmax"] for s in st)
    amax = max(s["amax"] for s in st)
    if vmax > FLIGHT_MAX_VEL_DEG_S or amax > FLIGHT_ACCEL_DEG_S2:
        need = a.slowdown * max(vmax / FLIGHT_MAX_VEL_DEG_S, math.sqrt(amax / FLIGHT_ACCEL_DEG_S2), 1.0)
        fail(f"Bahn zu schnell: max. {vmax:.1f}°/s (erlaubt {FLIGHT_MAX_VEL_DEG_S:.0f}), "
             f"max. {amax:.1f}°/s² (erlaubt {FLIGHT_ACCEL_DEG_S2:.0f}). "
             f"Mit --slowdown {math.ceil(need * 10) / 10:.1f} (oder mehr) zeitlich strecken - "
             f"aber nur, solange die Dauer unter {FLIGHT_MAX_DURATION_S:.0f} s bleibt.")

    urad = [[int(round(v * URAD_PER_DEG)) for v in c] for c in cols]
    note = f", zeitlich gestreckt x{a.slowdown:g}" if a.slowdown != 1.0 else ""
    crc = write_flight_header(Path(a.output), urad, nodes, Path(a.csv).name, joints, note)

    print(f"Flug-Tabelle: {a.output}")
    print(f"  {len(cols[0])} Punkte = {duration:.1f} s, {len(cols)} Gelenk(e), CRC32 0x{crc:08X}")
    for s, c, j, nd in zip(st, cols, joints, nodes):
        print(f"  Spalte {j} -> Node {nd}: Start/Ende {c[0]:+.2f}°, Bereich {s['min']:+.2f}° .. "
              f"{s['max']:+.2f}°, max. {s['vmax']:.1f}°/s, max. {s['amax']:.1f}°/s²")
    print("Nächster Schritt: Firmware bauen und flashen (pio run -t upload).")
    print("Danach prüfen: Die GUI zeigt unter 'Flug-Trajektorie' Tabelle OK, "
          f"{len(cols)} Gelenke und CRC-Endbyte 0x{crc & 0xFF:02X}.")
    if a.plot:
        plot(Path(a.plot), cols, joints, 0, flight=True)


def cmd_flight_info(a: argparse.Namespace) -> None:
    h = read_flight_header(Path(a.header))
    if not h["valid"]:
        print("Platzhalter - keine Flug-Trajektorie eingebaut. Der Arm fährt im Flug NICHT.")
        return
    urad, nodes = h["urad"], h["nodes"]
    crc = table_crc(urad)
    n = len(urad[0])
    print(f"Flug-Tabelle {a.header}")
    print(f"  {n} Punkte = {(n - 1) * DT_S:.1f} s, {len(urad)} Gelenk(e)")
    print(f"  CRC32 im Header 0x{h['crc']:08X}, berechnet 0x{crc:08X} -> "
          f"{'OK' if crc == h['crc'] else 'FALSCH (Tabelle verändert!)'}")
    for k, nd in enumerate(nodes):
        deg = [v / URAD_PER_DEG for v in urad[k]]
        s = stats(deg)
        print(f"  Spalte {k + 1} -> Node {nd}: Start {deg[0]:+.2f}°, Ende {deg[-1]:+.2f}°, "
              f"Bereich {s['min']:+.2f}° .. {s['max']:+.2f}°, max. {s['vmax']:.1f}°/s")


def cmd_example(a: argparse.Namespace) -> None:
    """Beispiel im ROS-Format: 3 Gelenke, rad, 0,1 s, 40 s. Absolutwerte am
    Anfang zählen nicht (der OBC fährt relativ). Gelenk 1 schwenkt auf etwa
    ±18°, Gelenk 2 versetzt dazu auf etwa ±13°; beide starten und enden in
    Ruhe. Langsam genug für 10 °/s und 4 °/s²."""
    n = 401
    with open(a.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint1", "joint2", "joint3"])
        for i in range(n):
            s = i / (n - 1)                                  # 0..1
            win = math.sin(math.pi * s) ** 2                 # Start/Ende in Ruhe
            j1 = 0.4 + math.radians(28.0 * math.sin(2 * math.pi * s) * win)
            j2 = -0.2 + math.radians(15.0 * math.sin(4 * math.pi * s) * win)
            j3 = 1.0                                         # bleibt stehen
            w.writerow([f"{v:.6f}" for v in (j1, j2, j3)])
    print(f"Beispiel-CSV: {a.output} ({n} Zeilen, {n * DT_S:.1f} s, 3 Gelenke, rad)")


def post_arm(server: str, body: dict) -> None:
    req = urllib.request.Request(
        f"{server.rstrip('/')}/api/command/arm",
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            print(f"Server: {r.status} {r.read().decode(errors='replace')}")
    except urllib.error.HTTPError as e:
        fail(f"Server {e.code}: {e.read().decode(errors='replace')}")
    except urllib.error.URLError as e:
        fail(f"Server nicht erreichbar ({server}): {e.reason}")


def check_nodes(nodes: list[int]) -> None:
    if not 1 <= len(nodes) <= MAX_JOINTS:
        fail(f"1 bis {MAX_JOINTS} Nodes angeben, nicht {len(nodes)}.")
    if len(set(nodes)) != len(nodes):
        fail("Node doppelt angegeben.")
    if any(not 1 <= n <= 8 for n in nodes):
        fail("Nodes müssen 1..8 sein.")


def cmd_start(a: argparse.Namespace) -> None:
    check_nodes(a.nodes)
    have = header_joint_count(DEFAULT_HEADER)
    if have is not None and have != len(a.nodes):
        fail(f"Die Tabelle im Repo hat {have} Gelenk(e), angegeben sind {len(a.nodes)} Nodes. "
             f"(Der OBC prüft das ebenfalls - entscheidend ist die geflashte Tabelle.)")
    pairs = ", ".join(f"Spalte {k + 1} -> Node {n}" for k, n in enumerate(a.nodes))
    print(f"Starte Trajektorie: {pairs}. Der OBC führt sie nur in TEST aus.")
    print("Abbrechen: 'stop', GUI-STOPP oder TEST verlassen.")
    body = {"action": "traj_test", "joint": a.nodes[0]}
    if len(a.nodes) > 1:
        body["joint2"] = a.nodes[1]
    post_arm(a.server, body)


def cmd_stop(a: argparse.Namespace) -> None:
    for n in a.nodes:
        post_arm(a.server, {"action": "stop", "joint": n})


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("convert", help="ROS-CSV -> include/arm_test_trajectory.hpp")
    c.add_argument("csv")
    c.add_argument("--joints", type=int, nargs="+", default=[1],
                   help=f"CSV-Spalten (1-basiert), 1 bis {MAX_JOINTS} Stück, Standard 1")
    c.add_argument("--unit", choices=["rad", "deg"], default="rad", help="Einheit der CSV, Standard rad")
    c.add_argument("--slowdown", type=float, default=1.0, help="zeitlich strecken, z. B. 2 = halb so schnell")
    c.add_argument("-o", "--output", default=str(DEFAULT_HEADER))
    c.add_argument("--plot", help="PNG mit Winkel- und Geschwindigkeitsverlauf")
    c.set_defaults(func=cmd_convert)

    fl = sub.add_parser("flight", help="Flug-CSV (absolute Winkel) -> include/arm_flight_trajectory.hpp")
    fl.add_argument("csv")
    fl.add_argument("--joints", type=int, nargs="+", required=True,
                    help=f"CSV-Spalten (1-basiert), 1 bis {FLIGHT_MAX_JOINTS} Stück")
    fl.add_argument("--nodes", type=int, nargs="+", required=True,
                    help="RCU-Node-ID je Spalte, gleiche Reihenfolge wie --joints")
    fl.add_argument("--unit", choices=["rad", "deg"], default="rad", help="Einheit der CSV, Standard rad")
    fl.add_argument("--slowdown", type=float, default=1.0, help="zeitlich strecken, z. B. 2 = halb so schnell")
    fl.add_argument("-o", "--output", default=str(FLIGHT_HEADER))
    fl.add_argument("--plot", help="PNG mit Winkel- und Geschwindigkeitsverlauf")
    fl.set_defaults(func=cmd_flight)

    fi = sub.add_parser("flight-info", help="eingebaute Flug-Tabelle anzeigen und CRC prüfen")
    fi.add_argument("--header", default=str(FLIGHT_HEADER))
    fi.set_defaults(func=cmd_flight_info)

    e = sub.add_parser("example", help="Beispiel-CSV im ROS-Format erzeugen")
    e.add_argument("-o", "--output", default="traj_example.csv")
    e.set_defaults(func=cmd_example)

    for name, func, txt in (("start", cmd_start, "Fahrt starten (nur TEST)"),
                            ("stop", cmd_stop, "Gelenke anhalten")):
        s = sub.add_parser(name, help=txt)
        s.add_argument("--nodes", type=int, nargs="+", required=True,
                       help="RCU-Node-IDs, in der Reihenfolge der Tabellenspalten")
        s.add_argument("--server", default=DEFAULT_SERVER)
        s.set_defaults(func=func)

    a = p.parse_args()
    if getattr(a, "slowdown", 1.0) < 1.0:
        fail("--slowdown muss >= 1 sein (nur langsamer, nie schneller).")
    a.func(a)


if __name__ == "__main__":
    main()
