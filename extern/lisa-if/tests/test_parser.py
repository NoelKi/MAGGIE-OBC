from pathlib import Path
import unittest

from lisa import (
    MessageId,
    ParseIssueKind,
    StreamParser,
    load_data,
    parse_stream,
    parse_stream_detailed,
)

from tests.helpers import make_frame


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class ParserTests(unittest.TestCase):
    def test_current_sample_json_is_a_regression_fixture(self) -> None:
        result = parse_stream_detailed(load_data(PROJECT_ROOT / "sample_data.json"))

        self.assertEqual(len(result.frames), 273)
        self.assertEqual(result.restored_frames, 51)
        self.assertEqual(result.crc_errors, 0)
        self.assertEqual(result.unknown_messages, 0)
        self.assertEqual(result.discarded_bytes, 0)
        self.assertEqual(result.incomplete_tail, b"")
        self.assertTrue(
            all(frame.message_id is MessageId.RCU_IMU_DATA for frame in result.frames)
        )

    def test_noise_is_discarded_and_reported(self) -> None:
        packet = make_frame()
        result = parse_stream_detailed(b"\x00\xff" + packet)
        self.assertEqual(len(result.frames), 1)
        self.assertEqual(result.discarded_bytes, 2)

    def test_missing_cr_is_recovered_in_every_frame_section(self) -> None:
        cases = [
            (make_frame(node_id=0x0D), 1),
            (make_frame(MessageId.SET_NODE_ID), 2),
            (make_frame(reserved=0x0D), 3),
            (make_frame(payload=b"\x0d" + bytes(11)), 4),
            self._packet_with_cr_in_crc(),
        ]
        sentinel = make_frame(payload=bytes([1]) + bytes(11))

        for packet, remove_at in cases:
            with self.subTest(remove_at=remove_at):
                self.assertEqual(packet[remove_at], 0x0D)
                damaged = packet[:remove_at] + packet[remove_at + 1 :]
                result = parse_stream_detailed(damaged + sentinel)
                self.assertEqual(len(result.frames), 2)
                self.assertTrue(result.frames[0].restored_cr)
                self.assertFalse(result.frames[1].restored_cr)

    def test_recovery_can_be_disabled(self) -> None:
        packet = make_frame(node_id=0x0D)
        damaged = packet[:1] + packet[2:]
        sentinel = make_frame()
        frames = parse_stream(damaged + sentinel, auto_recover_cr=False)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].node_id, 0)

    def test_crc_error_is_reported(self) -> None:
        packet = bytearray(make_frame())
        packet[4] ^= 1
        result = parse_stream_detailed(bytes(packet) + make_frame())
        self.assertEqual(len(result.frames), 1)
        self.assertEqual(result.crc_errors, 1)
        self.assertEqual(result.issues[0].kind, ParseIssueKind.CRC_ERROR)

    def test_incomplete_tail_is_preserved(self) -> None:
        packet = make_frame()
        result = parse_stream_detailed(packet[:-3])
        self.assertEqual(result.frames, [])
        self.assertEqual(result.incomplete_tail, packet[:-3])
        self.assertEqual(result.issues[-1].kind, ParseIssueKind.INCOMPLETE_FRAME)

    def test_stream_parser_preserves_frame_across_chunks(self) -> None:
        packet = make_frame()
        parser = StreamParser()
        first = parser.feed(packet[:7])
        second = parser.feed(packet[7:])
        final = parser.flush()

        self.assertEqual(first.frames, [])
        self.assertEqual(first.incomplete_tail, packet[:7])
        self.assertEqual(len(second.frames), 1)
        self.assertEqual(final.frames, [])
        self.assertEqual(parser.buffered_bytes, 0)

    def test_stream_parser_does_not_recover_a_delayed_final_cr(self) -> None:
        packet, crc_position = self._packet_with_cr_in_crc(final_crc_byte=True)
        self.assertEqual(crc_position, len(packet) - 1)
        parser = StreamParser()

        partial = parser.feed(packet[:-1])
        completed = parser.feed(packet[-1:])

        self.assertEqual(partial.frames, [])
        self.assertEqual(parser.buffered_bytes, 0)
        self.assertEqual(len(completed.frames), 1)
        self.assertFalse(completed.frames[0].restored_cr)

    @staticmethod
    def _packet_with_cr_in_crc(
        *, final_crc_byte: bool = False
    ) -> tuple[bytes, int]:
        for value in range(0x10000):
            payload = value.to_bytes(2, "little") + bytes(10)
            packet = make_frame(payload=payload)
            positions = [
                position
                for position in range(len(packet) - 4, len(packet))
                if packet[position] == 0x0D
            ]
            if final_crc_byte and packet[-1] == 0x0D:
                return packet, len(packet) - 1
            if not final_crc_byte and positions:
                return packet, positions[0]
        raise AssertionError("Kein deterministischer CRC-Testframe gefunden")


if __name__ == "__main__":
    unittest.main()
