"""Kommandozeilenoberflaeche fuer den LISA-UART-Parser."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import sys
from typing import Sequence

from .loader import load_data
from .parser import ParseResult, parse_stream_detailed
from .payloads import RcuImuPayload
from .plotter import plot_imu_data
from .protocol import GRAVITY_MS2


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lisa-parse",
        description="LISA-ODISEE UART-Telemetriedaten dekodieren",
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        help="JSON-, HEX-, TXT- oder Binaerdatei (Standard: sample_data.json)",
    )
    parser.add_argument(
        "--no-recover-cr",
        action="store_true",
        help="keine verlorenen 0x0D-Bytes anhand der CRC rekonstruieren",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="alle RCU-IMU-Messwerte statt einer Stichprobe anzeigen",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="RCU-IMU-Telemetriedaten in einem interaktiven Diagramm anzeigen",
    )
    parser.add_argument(
        "--save-plot",
        type=Path,
        metavar="PATH",
        help="IMU-Diagramm als Bilddatei (z. B. plot.png) speichern",
    )
    parser.add_argument(
        "--gyro-scale-mode",
        choices=["icd", "datasheet"],
        default="icd",
        help="Gyroskop-Skalierungsmodus: 'icd' (500/32768) oder 'datasheet' (ST LSM6DS3 17.5 mdps/LSB)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        raw_bytes = load_data(args.input)
        result = parse_stream_detailed(
            raw_bytes, auto_recover_cr=not args.no_recover_cr
        )
    except (FileNotFoundError, OSError, ValueError) as error:
        print(f"Fehler beim Einlesen: {error}", file=sys.stderr)
        return 2

    source_name = str(args.input) if args.input else "sample_data.json (Standard)"
    _print_summary(result, source_name, len(raw_bytes))
    _print_imu_table(result, show_all=args.all)

    if args.plot or args.save_plot:
        save_path_str = str(args.save_plot) if args.save_plot else None
        try:
            plot_imu_data(
                result,
                save_path=save_path_str,
                show=args.plot,
                title=f"LISA RCU-IMU Telemetrie ({source_name})",
                gyro_scale_mode=args.gyro_scale_mode,
            )
            if save_path_str:
                print(f"\nPlot erfolgreich gespeichert unter: {save_path_str}")
        except Exception as error:
            print(f"Fehler beim Erstellen des Plots: {error}", file=sys.stderr)
            return 3


    return 0 if result.frames else 1



def _print_summary(result: ParseResult, source_name: str, byte_count: int) -> None:
    type_counts = Counter(frame.name for frame in result.frames)
    print("=" * 128)
    print(" LISA-ODISEE TELEMETRIE-AUSWERTUNG (ICD v0.9)")
    print(f" Datenquelle       : {source_name}")
    print(f" Eingelesene Bytes : {byte_count}")
    print(f" Gueltige Pakete   : {len(result.frames)}")
    print(f" CR rekonstruiert  : {result.restored_frames}")
    print(f" CRC-Fehler        : {result.crc_errors}")
    print(f" Unbekannte IDs    : {result.unknown_messages}")
    print(f" Verworfene Bytes  : {result.discarded_bytes}")
    if result.incomplete_tail:
        print(f" Unvollst. Rest    : {len(result.incomplete_tail)} Bytes")
    if type_counts:
        counts = ", ".join(f"{name}={count}" for name, count in type_counts.items())
        print(f" Nachrichtentypen  : {counts}")
    print("=" * 128)


def _print_imu_table(result: ParseResult, *, show_all: bool) -> None:
    indexed_imu_frames = [
        (index, frame.decoded_payload)
        for index, frame in enumerate(result.frames, start=1)
        if isinstance(frame.decoded_payload, RcuImuPayload)
    ]
    if not indexed_imu_frames:
        print("Keine RCU-IMU-Daten enthalten.")
        return

    print(
        f"{'Nr.':>4} | {'Acc X [g]':>10} | {'Acc Y [g]':>10} | "
        f"{'Acc Z [g]':>10} | {'Gyro X [deg/s]':>15} | "
        f"{'Gyro Y [deg/s]':>15} | {'Gyro Z [deg/s]':>15}"
    )
    print("-" * 128)

    rows: list[tuple[int, RcuImuPayload] | None]
    if show_all or len(indexed_imu_frames) <= 15:
        rows = list(indexed_imu_frames)
    else:
        rows = indexed_imu_frames[:10] + [None] + indexed_imu_frames[-5:]

    for row in rows:
        if row is None:
            omitted = len(indexed_imu_frames) - 15
            print(f" ... | ... ({omitted} weitere RCU-IMU-Pakete ausgelassen) ...")
            continue
        index, payload = row
        ax, ay, az = payload.acc_g
        gx, gy, gz = payload.gyro_dps
        print(
            f"{index:4d} | {ax:+10.4f} | {ay:+10.4f} | {az:+10.4f} | "
            f"{gx:+15.3f} | {gy:+15.3f} | {gz:+15.3f}"
        )

    print("-" * 128)
    _print_imu_averages([payload for _, payload in indexed_imu_frames])
    print("=" * 128)


def _print_imu_averages(payloads: list[RcuImuPayload]) -> None:
    count = len(payloads)
    avg_acc = tuple(
        sum(payload.acc_g[axis] for payload in payloads) / count
        for axis in range(3)
    )
    avg_gyro = tuple(
        sum(payload.gyro_dps[axis] for payload in payloads) / count
        for axis in range(3)
    )
    avg_magnitude = sum(payload.acc_magnitude_g for payload in payloads) / count
    print("\n[DURCHSCHNITTSWERTE]")
    print(
        "  Beschleunigung : "
        f"X={avg_acc[0]:+.4f} g, Y={avg_acc[1]:+.4f} g, "
        f"Z={avg_acc[2]:+.4f} g (|a|={avg_magnitude:.4f} g "
        f"~= {avg_magnitude * GRAVITY_MS2:.2f} m/s^2)"
    )
    print(
        "  Drehraten      : "
        f"X={avg_gyro[0]:+.3f} deg/s, Y={avg_gyro[1]:+.3f} deg/s, "
        f"Z={avg_gyro[2]:+.3f} deg/s"
    )
