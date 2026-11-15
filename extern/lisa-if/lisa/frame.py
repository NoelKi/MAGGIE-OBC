"""Frame-Modell fuer dekodierte LISA-UART-Nachrichten."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .payloads import (
    DecodedPayload,
    JointStatePayload,
    RawPayload,
    RcuImuPayload,
    TemperaturePayload,
    Vector3,
    VoltagePayload,
)
from .protocol import MessageId, get_message_spec


@dataclass(frozen=True, slots=True)
class Frame:
    """Ein gepruefter UART-Frame mit roher und typisierter Payload."""

    node_id: int
    msg_id: int
    payload: bytes
    crc_valid: bool
    restored_cr: bool = False
    name: str = field(init=False)
    decoded_payload: DecodedPayload = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not 0 <= self.node_id <= 0xFF:
            raise ValueError("node_id muss zwischen 0 und 255 liegen")
        if not 0 <= self.msg_id <= 0xFF:
            raise ValueError("msg_id muss zwischen 0 und 255 liegen")

        raw_payload = bytes(self.payload)
        object.__setattr__(self, "payload", raw_payload)

        try:
            message_id = MessageId(self.msg_id)
        except ValueError:
            object.__setattr__(self, "name", f"UNKNOWN_0x{self.msg_id:02X}")
            object.__setattr__(self, "decoded_payload", RawPayload(raw_payload))
            return

        spec = get_message_spec(message_id)
        assert spec is not None
        if len(raw_payload) != spec.payload_size:
            raise ValueError(
                f"{message_id.name} erwartet {spec.payload_size} Payload-Bytes, "
                f"erhalten: {len(raw_payload)}"
            )

        object.__setattr__(self, "name", message_id.name)
        object.__setattr__(self, "decoded_payload", spec.decoder(raw_payload))

    @property
    def message_id(self) -> MessageId | None:
        try:
            return MessageId(self.msg_id)
        except ValueError:
            return None

    @property
    def acc_g(self) -> Vector3 | None:
        payload = self.decoded_payload
        return payload.acc_g if isinstance(payload, RcuImuPayload) else None

    @property
    def acc_ms2(self) -> Vector3 | None:
        payload = self.decoded_payload
        return payload.acc_ms2 if isinstance(payload, RcuImuPayload) else None

    @property
    def acc_magnitude_g(self) -> float | None:
        payload = self.decoded_payload
        return payload.acc_magnitude_g if isinstance(payload, RcuImuPayload) else None

    @property
    def gyro_dps(self) -> Vector3 | None:
        payload = self.decoded_payload
        return payload.gyro_dps if isinstance(payload, RcuImuPayload) else None

    @property
    def gyro_rads(self) -> Vector3 | None:
        payload = self.decoded_payload
        return payload.gyro_rads if isinstance(payload, RcuImuPayload) else None

    @property
    def gyro_magnitude_dps(self) -> float | None:
        payload = self.decoded_payload
        return payload.gyro_magnitude_dps if isinstance(payload, RcuImuPayload) else None

    @property
    def details(self) -> dict[str, Any]:
        """Kompatible Dictionary-Sicht fuer Nicht-IMU-Payloads."""

        payload = self.decoded_payload
        if isinstance(payload, JointStatePayload):
            return {
                "state": payload.state,
                "angle_urad": payload.angle_urad,
                "angle_deg": payload.angle_deg,
            }
        if isinstance(payload, VoltagePayload):
            return {
                "bus_voltage_v": payload.bus_voltage_v,
                "stepper_voltage_v": payload.stepper_voltage_v,
            }
        if isinstance(payload, TemperaturePayload):
            return {
                "mcu_temp_c": payload.mcu_temp_c,
                "imu_temp_c": payload.imu_temp_c,
            }
        return {}
