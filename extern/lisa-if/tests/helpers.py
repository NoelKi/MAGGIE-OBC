"""Gemeinsame Erzeugung gueltiger Testframes."""

from lisa import MessageId, calculate_crc32
from lisa.protocol import get_message_spec


def make_frame(
    message_id: MessageId = MessageId.RCU_IMU_DATA,
    payload: bytes | None = None,
    *,
    node_id: int = 0,
    reserved: int = 0,
) -> bytes:
    spec = get_message_spec(message_id)
    assert spec is not None
    if payload is None:
        payload = bytes(spec.payload_size)
    if len(payload) != spec.payload_size:
        raise ValueError("Falsche Test-Payloadlaenge")
    body = bytes((0xAA, node_id, message_id, reserved)) + payload
    return body + calculate_crc32(body).to_bytes(4, "little")
