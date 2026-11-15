#!/usr/bin/env python3
"""
Trajektorien-Test an EINEM Gelenk des Roboterarms.

Die ROS-Simulation liefert eine CSV: eine Zeile pro Zeitschritt (0,1 s), eine
Spalte pro Gelenk (joint 1, joint 2, ...), Winkel in rad. Dieses Skript

  1. wandelt EINE Spalte in die Tabelle include/arm_test_trajectory.hpp um, die
     der OBC an Bord abspielt (ArmTrajPlayer, nur im TEST-Zustand), und
  2. startet bzw. stoppt die Fahrt über den MAGGIE-Server.

Warum an Bord und nicht vom Boden gestreamt: Der Uplink kennt nur relative
Jog-Schritte in ganzen Grad, und die Laufzeit über das RXSM schwankt. Der OBC
schickt die Ziele dagegen exakt alle 100 ms als absolutes MOVE_ABS.

Die Tabelle enthält Winkel RELATIV zum ersten Punkt. Der OBC addiert sie zum
Winkel, den das Gelenk beim Start meldet - die Fahrt läuft also um den
aktuellen Stand herum, nicht auf die absoluten Winkel der Simulation.

Grenzen (prüfen Skript UND Firmware):
  - jeder Punkt innerhalb ±30° um den Start
  - Endpunkt = Startpunkt (fehlt das, hängt das Skript eine sanfte Rückfahrt an)
  - höchstens 20 °/s (sonst mit --slowdown zeitlich strecken)

Beispiele
---------
  # Beispiel-CSV erzeugen (3 Gelenke, Gelenk 1 bewegt sich)
  python tools/arm_trajectory.py example -o traj_example.csv

  # Spalte 1 in die Firmware-Tabelle umwandeln (+ Plot, falls matplotlib da ist)
  python tools/arm_trajectory.py convert traj_example.csv --joint 1 --plot traj.png

  # danach: Firmware bauen und flashen (pio run -t upload)

  # In der GUI TEST betreten, dann Fahrt an Node 8 starten / stoppen
  python tools/arm_trajectory.py start --node 8
  python tools/arm_trajectory.py stop  --node 8
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import urllib.error
import urllib.request
from pathlib import Path

# ── Muss zu ArmTrajPlayer (include/arm_traj_player.hpp) passen ──────────────
DT_S = 0.1                 # Takt der ROS-CSV und des Players
MAX_DEG = 30.0             # größte Auslenkung um den Start
MAX_VEL_DEG_S = 20.0       # höchste Geschwindigkeit
ACCEL_DEG_S2 = 40.0        # Beschleunigung des Players (nur Hinweis)
RETURN_VEL_DEG_S = 5.0     # Spitzengeschwindigkeit der angehängten Rückfahrt
MAX_POINTS = 6000          # 10 min - Tabelle liegt im Flash, 4 Byte/Punkt

URAD_PER_DEG = math.pi / 180.0 * 1e6

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_HEADER = ROOT / "include" / "arm_test_trajectory.hpp"
DEFAULT_SERVER = "http://localhost:3000"


def fail(msg: str) -> None:
    print(f"FEHLER: {msg}", file=sys.stderr)
    sys.exit(1)


# ── CSV lesen ────────────────────────────────────────────────────────────────

def read_column(path: Path, joint: int, unit: str) -> list[float]:
    """Spalte `joint` (1-basiert) in Grad. Kopfzeilen werden übersprungen."""
    values: list[float] = []
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
                if values:
                    fail(f"Zeile {lineno}: keine Zahl ({row}) - mitten in den Daten.")
                continue   # Kopfzeile
            if joint > len(nums):
                fail(f"Zeile {lineno} hat nur {len(nums)} Spalten, --joint {joint} gibt es nicht.")
            values.append(nums[joint - 1])
    if len(values) < 2:
        fail("Weniger als 2 Datenzeilen in der CSV.")
    if unit == "rad":
        values = [math.degrees(v) for v in values]
    return values


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


def append_return(offsets: list[float]) -> tuple[list[float], int]:
    """Sanfte Rückfahrt (Kosinus) vom letzten Punkt auf 0, Spitze RETURN_VEL."""
    last = offsets[-1]
    if abs(last) < 0.01:
        return offsets, 0
    duration = max(1.0, abs(last) * math.pi / (2.0 * RETURN_VEL_DEG_S))
    n = math.ceil(duration / DT_S)
    tail = [last * (1.0 + math.cos(math.pi * i / n)) / 2.0 for i in range(1, n + 1)]
    return offsets + tail, n


def check(offsets: list[float]) -> dict:
    vel = [(b - a) / DT_S for a, b in zip(offsets, offsets[1:])]
    acc = [(b - a) / DT_S for a, b in zip(vel, vel[1:])]
    return {
        "min": min(offsets),
        "max": max(offsets),
        "vmax": max((abs(v) for v in vel), default=0.0),
        "amax": max((abs(a) for a in acc), default=0.0),
        "vel": vel,
    }


# ── Header schreiben ─────────────────────────────────────────────────────────

def write_header(path: Path, urad: list[int], source: str, joint: int, note: str) -> None:
    lines = []
    for i in range(0, len(urad), 8):
        lines.append("    " + ", ".join(f"{v:9d}" for v in urad[i:i + 8]) + ",")
    body = "\n".join(lines)
    path.write_text(f"""#pragma once
#include <Arduino.h>

// ERZEUGT von tools/arm_trajectory.py - nicht von Hand aendern.
// Quelle: {source}, Spalte {joint}{note}
// {len(urad)} Punkte a {int(DT_S * 1000)} ms = {len(urad) * DT_S:.1f} s.
// Winkel in urad RELATIV zum Startwinkel des Gelenks; erster und letzter
// Punkt sind 0 (Ende = Start). Abgespielt von ArmTrajPlayer.

static constexpr uint32_t ARM_TRAJ_DT_MS = {int(DT_S * 1000)};
static constexpr uint16_t ARM_TRAJ_LEN   = {len(urad)};

static const int32_t ARM_TRAJ_OFFSET_URAD[ARM_TRAJ_LEN] PROGMEM = {{
{body}
}};
""")


def plot(path: Path, offsets: list[float], vel: list[float], n_return: int) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Hinweis: matplotlib fehlt - kein Plot.")
        return
    t = [i * DT_S for i in range(len(offsets))]
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 5))
    a1.plot(t, offsets, lw=1.5)
    for y in (MAX_DEG, -MAX_DEG):
        a1.axhline(y, color="r", ls="--", lw=0.8)
    if n_return:
        a1.axvspan(t[-n_return - 1], t[-1], color="0.9", label="angehängte Rückfahrt")
        a1.legend(loc="upper right")
    a1.set_ylabel("Winkel rel. zum Start [°]")
    a2.plot(t[1:], vel, lw=1.2)
    for y in (MAX_VEL_DEG_S, -MAX_VEL_DEG_S):
        a2.axhline(y, color="r", ls="--", lw=0.8)
    a2.set_ylabel("Geschwindigkeit [°/s]")
    a2.set_xlabel("Zeit [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"Plot: {path}")


# ── Befehle ──────────────────────────────────────────────────────────────────

def cmd_convert(a: argparse.Namespace) -> None:
    deg = read_column(Path(a.csv), a.joint, a.unit)
    deg = stretch(deg, a.slowdown)
    offsets = [v - deg[0] for v in deg]

    lo, hi = min(offsets), max(offsets)
    if lo < -MAX_DEG or hi > MAX_DEG:
        fail(f"Trajektorie reicht von {lo:+.2f}° bis {hi:+.2f}° um den Start - "
             f"erlaubt sind ±{MAX_DEG:.0f}°. Bitte in ROS kleiner planen.")

    offsets, n_return = append_return(offsets)
    offsets[0] = 0.0
    offsets[-1] = 0.0
    if len(offsets) > MAX_POINTS:
        fail(f"{len(offsets)} Punkte - mehr als {MAX_POINTS} ({MAX_POINTS * DT_S / 60:.0f} min).")

    st = check(offsets)
    if st["vmax"] > MAX_VEL_DEG_S:
        need = st["vmax"] / MAX_VEL_DEG_S
        fail(f"Höchstgeschwindigkeit {st['vmax']:.1f}°/s > {MAX_VEL_DEG_S:.0f}°/s. "
             f"Mit --slowdown {math.ceil(need * 10) / 10:.1f} (oder mehr) zeitlich strecken.")

    urad = [int(round(v * URAD_PER_DEG)) for v in offsets]
    note = f", zeitlich gestreckt x{a.slowdown:g}" if a.slowdown != 1.0 else ""
    write_header(Path(a.output), urad, Path(a.csv).name, a.joint, note)

    print(f"Tabelle: {a.output}")
    print(f"  {len(urad)} Punkte = {len(urad) * DT_S:.1f} s")
    print(f"  Bereich {st['min']:+.2f}° .. {st['max']:+.2f}° um den Start")
    print(f"  max. {st['vmax']:.1f}°/s, max. {st['amax']:.0f}°/s²")
    if n_return:
        print(f"  Ende != Start: Rückfahrt mit {n_return} Punkten ({n_return * DT_S:.1f} s) angehängt.")
    if st["amax"] > ACCEL_DEG_S2:
        print(f"  Hinweis: Beschleunigung bis {st['amax']:.0f}°/s², der Player fährt mit "
              f"{ACCEL_DEG_S2:.0f}°/s² - an diesen Stellen läuft das Gelenk etwas nach.")
    print("Nächster Schritt: Firmware bauen und flashen (pio run -t upload).")

    if a.plot:
        plot(Path(a.plot), offsets, st["vel"], n_return)


def cmd_example(a: argparse.Namespace) -> None:
    """Beispiel im ROS-Format: 3 Gelenke, rad, 0,1 s, 20 s. Gelenk 1 startet
    bei 0,4 rad (absolut - zählt nicht, der OBC fährt relativ), schwenkt
    sanft auf etwa +18°, dann -18° und kommt in Ruhe zum Start zurück."""
    n = 201
    rows = []
    for i in range(n):
        s = i / (n - 1)                                # 0..1
        # sin(2πs) = einmal hin und her, sin²(πs) = Start und Ende in Ruhe
        j1 = 0.4 + math.radians(28.0 * math.sin(2 * math.pi * s) * math.sin(math.pi * s) ** 2)
        j2 = -0.2 + 0.1 * s                            # andere Gelenke: nur fürs Format
        j3 = 1.0
        rows.append((j1, j2, j3))
    with open(a.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint1", "joint2", "joint3"])
        for r in rows:
            w.writerow([f"{v:.6f}" for v in r])
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


def cmd_start(a: argparse.Namespace) -> None:
    print(f"Starte Trajektorie an Node {a.node}. Der OBC führt sie nur in TEST aus.")
    print("Abbrechen: 'stop', GUI-STOPP oder TEST verlassen.")
    post_arm(a.server, {"action": "traj_test", "joint": a.node})


def cmd_stop(a: argparse.Namespace) -> None:
    post_arm(a.server, {"action": "stop", "joint": a.node})


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("convert", help="ROS-CSV -> include/arm_test_trajectory.hpp")
    c.add_argument("csv")
    c.add_argument("--joint", type=int, default=1, help="Spalte (1-basiert), Standard 1")
    c.add_argument("--unit", choices=["rad", "deg"], default="rad", help="Einheit der CSV, Standard rad")
    c.add_argument("--slowdown", type=float, default=1.0, help="zeitlich strecken, z. B. 2 = halb so schnell")
    c.add_argument("-o", "--output", default=str(DEFAULT_HEADER))
    c.add_argument("--plot", help="PNG mit Winkel- und Geschwindigkeitsverlauf")
    c.set_defaults(func=cmd_convert)

    e = sub.add_parser("example", help="Beispiel-CSV im ROS-Format erzeugen")
    e.add_argument("-o", "--output", default="traj_example.csv")
    e.set_defaults(func=cmd_example)

    for name, func, txt in (("start", cmd_start, "Fahrt am Gelenk starten (nur TEST)"),
                            ("stop", cmd_stop, "Gelenk anhalten")):
        s = sub.add_parser(name, help=txt)
        s.add_argument("--node", type=int, required=True, help="RCU-Node-ID des Gelenks (1..8)")
        s.add_argument("--server", default=DEFAULT_SERVER)
        s.set_defaults(func=func)

    a = p.parse_args()
    if getattr(a, "slowdown", 1.0) < 1.0:
        fail("--slowdown muss >= 1 sein (nur langsamer, nie schneller).")
    a.func(a)


if __name__ == "__main__":
    main()
