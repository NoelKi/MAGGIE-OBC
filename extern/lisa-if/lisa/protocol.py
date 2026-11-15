"""Zentrale Protokolldefinitionen gemaess LISA-ODISEE ICD v0.9."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
from types import MappingProxyType
from typing import Callable, Mapping

from .payloads import (
    DecodedPayload,
    decode_empty,
    decode_joint_state,
    decode_raw,
    decode_rcu_imu,
    decode_temperature,
    decode_voltage,
)


SYNC_BYTE = 0xAA
HEADER_SIZE = 4
CRC_SIZE = 4


class MessageId(IntEnum):
    """Nachrichten-IDs fuer Steuerbefehle und Telemetrie."""

    STOP = 0x01
    INHIBIT = 0x02
    RELEASE = 0x03
    PAUSE = 0x04
    CONTINUE = 0x05
    STEPPER_MOVE_REL = 0x06
    STEPPER_MOVE_ABS = 0x07
    STEPPER_MOVE_STEPS = 0x08
    RESET = 0x09
    STEPPER_SET_VELOCITY = 0x0A
    SET_HOME = 0x0B
    INITIATE_LSS_SCAN = 0x0C
    SET_NODE_ID = 0x0D
    BLINK = 0x0E

    JOINT_STATE = 0x14
    IMU_DATA = 0x15
    VOLTAGE_DATA = 0x16
    TEMPERATURE_DATA = 0x17
    REFERENCE_ANGLE_DATA = 0x18
    IDENTIFIED = 0x19
    RCU_IMU_DATA = 0x1A
    RCU_FREEFALL_EVENT = 0x1B


Decoder = Callable[[bytes], DecodedPayload]


@dataclass(frozen=True, slots=True)
class MessageSpec:
    """Laenge und Decoder einer Nachricht an einer zentralen Stelle."""

    payload_size: int
    decoder: Decoder = decode_raw

    @property
    def frame_size(self) -> int:
        return HEADER_SIZE + self.payload_size + CRC_SIZE


def _spec(payload_size: int, decoder: Decoder = decode_raw) -> MessageSpec:
    return MessageSpec(payload_size, decoder)


MESSAGE_SPECS: Mapping[MessageId, MessageSpec] = MappingProxyType(
    {
        MessageId.STOP: _spec(0, decode_empty),
        MessageId.INHIBIT: _spec(0, decode_empty),
        MessageId.RELEASE: _spec(0, decode_empty),
        MessageId.PAUSE: _spec(0, decode_empty),
        MessageId.CONTINUE: _spec(0, decode_empty),
        MessageId.STEPPER_MOVE_REL: _spec(4),
        MessageId.STEPPER_MOVE_ABS: _spec(4),
        MessageId.STEPPER_MOVE_STEPS: _spec(4),
        MessageId.RESET: _spec(0, decode_empty),
        # ICD v0.9: SET_VELOCITY bekam ein zweites Feld (Acceleration) dazu -
        # Payload wuchs von 4 auf 8 Byte (Velocity uint32 + Acceleration uint32).
        MessageId.STEPPER_SET_VELOCITY: _spec(8),
        MessageId.SET_HOME: _spec(4),
        MessageId.INITIATE_LSS_SCAN: _spec(0, decode_empty),
        MessageId.SET_NODE_ID: _spec(16),
        MessageId.BLINK: _spec(0, decode_empty),
        # ICD v0.9: Payload wuchs von 4 auf 5 Byte (Winkel jetzt volles int32,
        # matcht CANopen-Objekt 0x6001 exakt statt auf 24 Bit abgeschnitten).
        MessageId.JOINT_STATE: _spec(5, decode_joint_state),
        MessageId.IMU_DATA: _spec(16),
        MessageId.VOLTAGE_DATA: _spec(4, decode_voltage),
        MessageId.TEMPERATURE_DATA: _spec(4, decode_temperature),
        MessageId.REFERENCE_ANGLE_DATA: _spec(4),
        MessageId.IDENTIFIED: _spec(16),
        MessageId.RCU_IMU_DATA: _spec(12, decode_rcu_imu),
        MessageId.RCU_FREEFALL_EVENT: _spec(0, decode_empty),
    }
)

# Rueckwaertskompatible Sicht fuer bestehenden Client-Code.
PAYLOAD_SIZES: Mapping[MessageId, int] = MappingProxyType(
    {message_id: spec.payload_size for message_id, spec in MESSAGE_SPECS.items()}
)


GRAVITY_MS2 = 9.80665

RCU_ACC_SCALE_G = 4.0 / 32768.0
RCU_ACC_SCALE_MS2 = RCU_ACC_SCALE_G * GRAVITY_MS2
RCU_GYRO_SCALE_DPS = 500.0 / 32768.0
RCU_GYRO_SCALE_RADS = RCU_GYRO_SCALE_DPS * (math.pi / 180.0)

JOINT_ACC_SCALE_G = 4.0 / 32768.0
JOINT_GYRO_SCALE_DPS = 125.0 / 32768.0


def get_message_spec(msg_id: int | MessageId) -> MessageSpec | None:
    """Liefert die Spezifikation oder ``None`` fuer eine unbekannte ID."""

    try:
        message_id = MessageId(msg_id)
    except ValueError:
        return None
    return MESSAGE_SPECS[message_id]
