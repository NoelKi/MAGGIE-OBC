"""Modul zum Extrahieren und Grafischen Plotten von IMU-Telemetriedaten."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Sequence

from .frame import Frame
from .parser import ParseResult
from .payloads import RcuImuPayload
from .protocol import (
    GRAVITY_MS2,
    RCU_ACC_SCALE_G,
    RCU_GYRO_SCALE_DPS,
)

if TYPE_CHECKING:
    import matplotlib.figure


# Sensordatenblatt ST LSM6DS3 Empfindlichkeiten (Typische Werte aus Table 3, DocID026899 Rev 8):
# LA_So (FS = ±4g): 0.122 mg/LSB = 0.000122 g/LSB
LSM6DS3_ACC_SENSITIVITY_4G = 0.000122
# G_So (FS = ±500dps): 17.50 mdps/LSB = 0.0175 dps/LSB
LSM6DS3_GYRO_SENSITIVITY_500DPS = 0.0175
# G_So (FS = ±125dps): 4.375 mdps/LSB = 0.004375 dps/LSB
LSM6DS3_GYRO_SENSITIVITY_125DPS = 0.004375


def extract_imu_series(
    data: Sequence[Frame] | ParseResult,
    *,
    acc_unit: str = "g",
    gyro_unit: str = "dps",
    gyro_scale_mode: str = "icd",
) -> dict[str, list[float]]:
    """Extrahiert Zeitreihen fuer Beschleunigung und Drehrate aus RCU-IMU-Frames.

    Args:
        data: Liste von Frames oder ParseResult.
        acc_unit: 'g' oder 'ms2'.
        gyro_unit: 'dps' (deg/s) oder 'rads' (rad/s).
        gyro_scale_mode: 'icd' (500/32768 dps/LSB) oder 'datasheet' (ST LSM6DS3 17.5 mdps/LSB).

    Returns:
        Dictionary mit Schlüsseln:
        'indices', 'acc_x', 'acc_y', 'acc_z', 'acc_mag', 'gyro_x', 'gyro_y', 'gyro_z', 'gyro_mag'
    """
    frames = data.frames if isinstance(data, ParseResult) else data

    indices: list[float] = []
    acc_x: list[float] = []
    acc_y: list[float] = []
    acc_z: list[float] = []
    acc_mag: list[float] = []

    gyro_x: list[float] = []
    gyro_y: list[float] = []
    gyro_z: list[float] = []
    gyro_mag: list[float] = []

    # Berechnungskorrekturfaktor falls Gyro-Datenblatt-Modus gewählt wird
    gyro_factor = 1.0
    if gyro_scale_mode == "datasheet":
        # ratio datasheet (0.0175 dps/LSB) zu icd (500/32768 dps/LSB)
        gyro_factor = LSM6DS3_GYRO_SENSITIVITY_500DPS / RCU_GYRO_SCALE_DPS

    import math

    for idx, frame in enumerate(frames):
        payload = frame.decoded_payload
        if isinstance(payload, RcuImuPayload):
            indices.append(float(idx))

            # Beschleunigung
            if acc_unit == "ms2":
                ax, ay, az = payload.acc_ms2
                amag = payload.acc_magnitude_g * GRAVITY_MS2
            else:
                ax, ay, az = payload.acc_g
                amag = payload.acc_magnitude_g

            acc_x.append(ax)
            acc_y.append(ay)
            acc_z.append(az)
            acc_mag.append(amag)

            # Drehraten
            gx, gy, gz = payload.gyro_dps
            if gyro_scale_mode == "datasheet":
                gx *= gyro_factor
                gy *= gyro_factor
                gz *= gyro_factor

            gmag = math.sqrt(gx * gx + gy * gy + gz * gz)

            if gyro_unit == "rads":
                rad_conv = math.pi / 180.0
                gx *= rad_conv
                gy *= rad_conv
                gz *= rad_conv
                gmag *= rad_conv

            gyro_x.append(gx)
            gyro_y.append(gy)
            gyro_z.append(gz)
            gyro_mag.append(gmag)

    return {
        "indices": indices,
        "acc_x": acc_x,
        "acc_y": acc_y,
        "acc_z": acc_z,
        "acc_mag": acc_mag,
        "gyro_x": gyro_x,
        "gyro_y": gyro_y,
        "gyro_z": gyro_z,
        "gyro_mag": gyro_mag,
    }


def plot_imu_data(
    data: Sequence[Frame] | ParseResult,
    *,
    save_path: str | None = None,
    show: bool = False,
    acc_unit: str = "g",
    gyro_unit: str = "dps",
    title: str = "LISA RCU-IMU Telemetriedaten",
    sample_rate_hz: float | None = None,
    gyro_scale_mode: str = "icd",
) -> Any:
    """Erstellt ein 2-Panel-Diagramm der RCU-IMU-Messwerte (Beschleunigung und Drehraten).

    Args:
        data: Liste von Frames oder ParseResult.
        save_path: Optionaler Dateipfad zum Speichern des Plots (.png, .svg, etc.).
        show: Ob plt.show() aufgerufen werden soll.
        acc_unit: 'g' oder 'ms2'.
        gyro_unit: 'dps' oder 'rads'.
        title: Haupttitel des Diagramms.
        sample_rate_hz: Abtastrate in Hz zur Umrechnung der X-Achse in Sekunden.
        gyro_scale_mode: 'icd' oder 'datasheet'.

    Returns:
        Matplotlib Figure Objekt.
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError as err:
        raise ImportError(
            "Matplotlib ist nicht installiert. Installiere es mit `pip install matplotlib` "
            "oder `pip install -e .[plot]`, um IMU-Plots zu erstellen."
        ) from err

    series = extract_imu_series(
        data, acc_unit=acc_unit, gyro_unit=gyro_unit, gyro_scale_mode=gyro_scale_mode
    )

    if not series["indices"]:
        raise ValueError("Keine RCU-IMU Telemetrie-Frames in den Daten enthalten.")

    x_vals = series["indices"]
    x_label = "Frame Index"
    if sample_rate_hz and sample_rate_hz > 0:
        x_vals = [i / sample_rate_hz for i in x_vals]
        x_label = "Zeit [s]"

    acc_unit_str = "g" if acc_unit == "g" else "m/s²"
    gyro_unit_str = "°/s" if gyro_unit == "dps" else "rad/s"

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # Subplot 1: Beschleunigung
    ax1.plot(x_vals, series["acc_x"], label="Acc X", color="#d62728", alpha=0.85)
    ax1.plot(x_vals, series["acc_y"], label="Acc Y", color="#2ca02c", alpha=0.85)
    ax1.plot(x_vals, series["acc_z"], label="Acc Z", color="#1f77b4", alpha=0.85)
    ax1.plot(
        x_vals,
        series["acc_mag"],
        label="|Acc| Betrag",
        color="#7f7f7f",
        linestyle="--",
        alpha=0.7,
    )
    ax1.set_ylabel(f"Beschleunigung [{acc_unit_str}]")
    ax1.set_title(title)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper right")

    # Subplot 2: Gyroskop
    ax2.plot(x_vals, series["gyro_x"], label="Gyro X (Pitch)", color="#d62728", alpha=0.85)
    ax2.plot(x_vals, series["gyro_y"], label="Gyro Y (Roll)", color="#2ca02c", alpha=0.85)
    ax2.plot(x_vals, series["gyro_z"], label="Gyro Z (Yaw)", color="#1f77b4", alpha=0.85)
    ax2.plot(
        x_vals,
        series["gyro_mag"],
        label="|Gyro| Betrag",
        color="#7f7f7f",
        linestyle="--",
        alpha=0.7,
    )
    ax2.set_xlabel(x_label)
    ax2.set_ylabel(f"Drehrate [{gyro_unit_str}]")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="upper right")

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")

    if show:
        plt.show()

    return fig
