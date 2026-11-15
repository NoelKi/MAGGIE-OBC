"""Typisierte Payloadmodelle und Decoder fuer das LISA-UART-Protokoll."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import TypeAlias


Vector3: TypeAlias = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class EmptyPayload:
    """Nachricht ohne Payload."""


@dataclass(frozen=True, slots=True)
class RawPayload:
    """Noch nicht fachlich dekodierte Payload."""

    data: bytes


@dataclass(frozen=True, slots=True)
class RcuImuPayload:
    """Dekodierte Messwerte der RCU-IMU."""

    acc_g: Vector3
    acc_ms2: Vector3
    acc_magnitude_g: float
    gyro_dps: Vector3
    gyro_rads: Vector3
    gyro_magnitude_dps: float


@dataclass(frozen=True, slots=True)
class JointStatePayload:
    """Zustand und vorzeichenbehafteter 32-Bit-Gelenkwinkel (ICD v0.9: vorher
    24 Bit - siehe decode_joint_state())."""

    state: int
    angle_urad: int
    angle_deg: float


@dataclass(frozen=True, slots=True)
class VoltagePayload:
    """Versorgungsspannungen in Volt."""

    bus_voltage_v: float
    stepper_voltage_v: float


@dataclass(frozen=True, slots=True)
class TemperaturePayload:
    """MCU- und IMU-Temperatur in Grad Celsius."""

    mcu_temp_c: float
    imu_temp_c: float


DecodedPayload: TypeAlias = (
    EmptyPayload
    | RawPayload
    | RcuImuPayload
    | JointStatePayload
    | VoltagePayload
    | TemperaturePayload
)


def decode_empty(payload: bytes) -> EmptyPayload:
    _require_size(payload, 0)
    return EmptyPayload()


def decode_raw(payload: bytes) -> RawPayload:
    return RawPayload(bytes(payload))


def decode_rcu_imu(payload: bytes) -> RcuImuPayload:
    _require_size(payload, 12)
    ax, ay, az, gx, gy, gz = struct.unpack("<hhhhhh", payload)

    # Der lokale Import verhindert einen Modulzyklus zur Spec-Registry.
    from .protocol import (
        RCU_ACC_SCALE_G,
        RCU_ACC_SCALE_MS2,
        RCU_GYRO_SCALE_DPS,
        RCU_GYRO_SCALE_RADS,
    )

    acc_g = tuple(value * RCU_ACC_SCALE_G for value in (ax, ay, az))
    acc_ms2 = tuple(value * RCU_ACC_SCALE_MS2 for value in (ax, ay, az))
    gyro_dps = tuple(value * RCU_GYRO_SCALE_DPS for value in (gx, gy, gz))
    gyro_rads = tuple(value * RCU_GYRO_SCALE_RADS for value in (gx, gy, gz))
    return RcuImuPayload(
        acc_g=acc_g,
        acc_ms2=acc_ms2,
        acc_magnitude_g=math.sqrt(sum(value**2 for value in acc_g)),
        gyro_dps=gyro_dps,
        gyro_rads=gyro_rads,
        gyro_magnitude_dps=math.sqrt(sum(value**2 for value in gyro_dps)),
    )


def decode_joint_state(payload: bytes) -> JointStatePayload:
    """ICD v0.9: Payload wuchs von 4 auf 5 Byte - der Winkel ist jetzt ein
    volles int32 (matcht CANopen-Objekt 0x6001 exakt, nicht mehr abgeschnitten
    auf 24 Bit wie in v0.7/v0.8)."""
    _require_size(payload, 5)
    state = payload[0]
    (angle_urad,) = struct.unpack("<i", payload[1:5])
    return JointStatePayload(
        state=state,
        angle_urad=angle_urad,
        angle_deg=(angle_urad / 1e6) * (180.0 / math.pi),
    )


def decode_voltage(payload: bytes) -> VoltagePayload:
    _require_size(payload, 4)
    bus_mv, stepper_mv = struct.unpack("<HH", payload)
    return VoltagePayload(bus_mv / 1000.0, stepper_mv / 1000.0)


def decode_temperature(payload: bytes) -> TemperaturePayload:
    """Skalierung /100 statt der im ICD dokumentierten /10 (0.1 degC/Count):
    reale RCU-Werte mit /10 lagen konsistent um Faktor 10 zu hoch (~800-900
    degC, physikalisch unmoeglich) - mit /100 ergeben sich plausible Werte
    (~80-90 degC). Nutzerbestaetigt/-angeordnet, weicht bewusst vom
    ICD-Text ab (Firmware-Diskrepanz, nicht unser Decoder)."""
    _require_size(payload, 4)
    mcu_deci_c, imu_deci_c = struct.unpack("<hh", payload)
    return TemperaturePayload(mcu_deci_c / 100.0, imu_deci_c / 100.0)


def _require_size(payload: bytes, expected: int) -> None:
    if len(payload) != expected:
        raise ValueError(
            f"Payload hat {len(payload)} Bytes; erwartet werden {expected} Bytes"
        )
