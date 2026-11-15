"""Parser, Diagnostik und inkrementelles Lesen fuer LISA-UART-Streams."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from .crc import verify_crc32
from .frame import Frame
from .protocol import (
    CRC_SIZE,
    HEADER_SIZE,
    MESSAGE_SPECS,
    SYNC_BYTE,
    MessageId,
    MessageSpec,
    get_message_spec,
)


class ParseIssueKind(str, Enum):
    CRC_ERROR = "crc_error"
    UNKNOWN_MESSAGE = "unknown_message"
    INCOMPLETE_FRAME = "incomplete_frame"


@dataclass(frozen=True, slots=True)
class ParseIssue:
    """Ein beim Synchronisieren verworfener oder unvollstaendiger Frame."""

    offset: int
    kind: ParseIssueKind
    message: str
    # Rohe Kandidaten-Bytes, sofern bekannt (z.B. bei CRC_ERROR: genau die
    # Bytes, die als Frame erwartet, aber wegen ungueltiger Pruefsumme
    # verworfen wurden) - fuer Debugging auf echter Hardware unverzichtbar,
    # sonst ist ein CRC-Fehler nicht von den tatsaechlich angekommenen Bytes
    # her nachvollziehbar.
    raw: bytes | None = None


@dataclass(slots=True)
class ParseResult:
    """Frames plus nachvollziehbare Parserdiagnostik."""

    frames: list[Frame] = field(default_factory=list)
    issues: list[ParseIssue] = field(default_factory=list)
    discarded_bytes: int = 0
    incomplete_tail: bytes = b""

    @property
    def crc_errors(self) -> int:
        return sum(issue.kind is ParseIssueKind.CRC_ERROR for issue in self.issues)

    @property
    def unknown_messages(self) -> int:
        return sum(
            issue.kind is ParseIssueKind.UNKNOWN_MESSAGE for issue in self.issues
        )

    @property
    def restored_frames(self) -> int:
        return sum(frame.restored_cr for frame in self.frames)


@dataclass(frozen=True, slots=True)
class _Recovery:
    frame: Frame
    consumed_bytes: int


def parse_stream(data: bytes, auto_recover_cr: bool = True) -> list[Frame]:
    """Kompatible Kurzform, die nur erfolgreich gepruefte Frames liefert."""

    return parse_stream_detailed(data, auto_recover_cr).frames


def parse_stream_detailed(
    data: bytes, auto_recover_cr: bool = True
) -> ParseResult:
    """Parst einen abgeschlossenen Bytestrom inklusive Fehlerdiagnostik."""

    result, _ = _parse_data(bytes(data), auto_recover_cr, final=True)
    return result


class StreamParser:
    """Zustandsbehafteter Parser fuer blockweise gelesene UART-Daten."""

    def __init__(self, auto_recover_cr: bool = True) -> None:
        self.auto_recover_cr = auto_recover_cr
        self._buffer = bytearray()
        self._stream_offset = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def feed(self, chunk: bytes) -> ParseResult:
        """Fuegt einen Datenblock hinzu und behaelt einen Teilframe intern."""

        self._buffer.extend(chunk)
        result, consumed = _parse_data(
            bytes(self._buffer), self.auto_recover_cr, final=False
        )
        result.issues = [
            replace(issue, offset=issue.offset + self._stream_offset)
            for issue in result.issues
        ]
        del self._buffer[:consumed]
        self._stream_offset += consumed
        result.incomplete_tail = bytes(self._buffer)
        return result

    def flush(self) -> ParseResult:
        """Schliesst den Stream ab und meldet einen verbleibenden Teilframe."""

        result, _ = _parse_data(
            bytes(self._buffer), self.auto_recover_cr, final=True
        )
        result.issues = [
            replace(issue, offset=issue.offset + self._stream_offset)
            for issue in result.issues
        ]
        self._stream_offset += len(self._buffer)
        self._buffer.clear()
        return result


def _parse_data(
    data: bytes, auto_recover_cr: bool, *, final: bool
) -> tuple[ParseResult, int]:
    result = ParseResult()
    idx = 0
    total_len = len(data)
    max_recovery_size = max(spec.frame_size - 1 for spec in MESSAGE_SPECS.values())

    while idx < total_len:
        if data[idx] != SYNC_BYTE:
            idx += 1
            result.discarded_bytes += 1
            continue

        available = total_len - idx
        if available < HEADER_SIZE:
            _set_incomplete_tail(result, data, idx, final)
            break

        msg_id = data[idx + 2]
        spec = get_message_spec(msg_id)

        if spec is not None and available >= spec.frame_size:
            packet = data[idx : idx + spec.frame_size]
            if _packet_crc_valid(packet):
                result.frames.append(_frame_from_packet(packet, spec))
                idx += spec.frame_size
                continue

        if auto_recover_cr:
            recovery = _recover_missing_cr(
                data, idx, require_next_sync=not final
            )
            if recovery is not None:
                result.frames.append(recovery.frame)
                idx += recovery.consumed_bytes
                continue

        if spec is None:
            # Bei einem offenen Stream koennte die unbekannte ID durch ein
            # fehlendes Header-Byte verschoben sein. Auf genug Daten warten.
            if not final and available < max_recovery_size:
                result.incomplete_tail = data[idx:]
                break
            result.issues.append(
                ParseIssue(
                    idx,
                    ParseIssueKind.UNKNOWN_MESSAGE,
                    f"Unbekannte Message-ID 0x{msg_id:02X}",
                )
            )
            idx += 1
            result.discarded_bytes += 1
            continue

        if available < spec.frame_size:
            _set_incomplete_tail(result, data, idx, final)
            break

        result.issues.append(
            ParseIssue(
                idx,
                ParseIssueKind.CRC_ERROR,
                f"CRC fuer {MessageId(msg_id).name} ungueltig",
                raw=bytes(data[idx : idx + spec.frame_size]),
            )
        )
        idx += 1
        result.discarded_bytes += 1

    return result, idx


def _recover_missing_cr(
    data: bytes, idx: int, *, require_next_sync: bool
) -> _Recovery | None:
    """Fuegt genau ein 0x0D an jeder moeglichen Frameposition testweise ein."""

    candidates: list[_Recovery] = []
    available = len(data) - idx

    for message_id, spec in MESSAGE_SPECS.items():
        observed_size = spec.frame_size - 1
        if available < observed_size:
            continue
        if require_next_sync and (
            available == observed_size
            or data[idx + observed_size] != SYNC_BYTE
        ):
            continue

        observed = data[idx : idx + observed_size]
        # Das Synchronisationsbyte ist 0xAA und kann daher nicht das verlorene
        # 0x0D sein. Alle uebrigen Positionen einschliesslich CRC sind moeglich.
        for insert_at in range(1, len(observed) + 1):
            packet = observed[:insert_at] + b"\x0d" + observed[insert_at:]
            if packet[2] != message_id or not _packet_crc_valid(packet):
                continue
            candidates.append(
                _Recovery(
                    _frame_from_packet(packet, spec, restored_cr=True),
                    observed_size,
                )
            )

    if not candidates:
        return None

    # Mehrere CRC-gueltige Rekonstruktionen sind extrem unwahrscheinlich. Eine
    # eindeutige Rekonstruktion wird bevorzugt; sonst die kuerzeste Nachricht.
    unique = {
        (candidate.consumed_bytes, candidate.frame.msg_id, candidate.frame.payload):
        candidate
        for candidate in candidates
    }
    return min(unique.values(), key=lambda candidate: candidate.consumed_bytes)


def _packet_crc_valid(packet: bytes) -> bool:
    if len(packet) < HEADER_SIZE + CRC_SIZE:
        return False
    body = packet[:-CRC_SIZE]
    expected_crc = int.from_bytes(packet[-CRC_SIZE:], "little")
    return verify_crc32(body, expected_crc)


def _frame_from_packet(
    packet: bytes, spec: MessageSpec, *, restored_cr: bool = False
) -> Frame:
    return Frame(
        node_id=packet[1],
        msg_id=packet[2],
        payload=packet[HEADER_SIZE : HEADER_SIZE + spec.payload_size],
        crc_valid=True,
        restored_cr=restored_cr,
    )


def _set_incomplete_tail(
    result: ParseResult, data: bytes, idx: int, final: bool
) -> None:
    result.incomplete_tail = data[idx:]
    if final:
        result.issues.append(
            ParseIssue(
                idx,
                ParseIssueKind.INCOMPLETE_FRAME,
                f"Unvollstaendiger Frame mit {len(result.incomplete_tail)} Bytes",
            )
        )
