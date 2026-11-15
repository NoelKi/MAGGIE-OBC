import json
from pathlib import Path
import tempfile
import unittest

from lisa import load_data
from lisa.loader import _extract_hex_bytes


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class LoaderTests(unittest.TestCase):
    def test_current_sample_json_is_loaded(self) -> None:
        data = load_data(PROJECT_ROOT / "sample_data.json")
        self.assertEqual(len(data), 5409)
        self.assertEqual(data[:4], bytes.fromhex("AA 00 1A 00"))

    def test_nested_json_data_is_found(self) -> None:
        nested = {"metadata": {"name": "capture"}, "capture": {"raw_hex": "AA 00"}}
        self.assertEqual(_extract_hex_bytes(nested), b"\xaa\x00")

    def test_byte_values_are_validated(self) -> None:
        with self.assertRaisesRegex(ValueError, "ausserhalb"):
            _extract_hex_bytes([0, 256])

    def test_invalid_json_is_not_reinterpreted_as_hex(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.json"
            path.write_text("{broken", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Ungueltiges JSON"):
                load_data(path)

    def test_valid_json_with_invalid_structure_has_clear_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.json"
            path.write_text(json.dumps({"metadata": 42}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Ungueltige UART-Daten"):
                load_data(path)

    def test_unknown_extension_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.csv"
            path.write_text("AA 00", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Nicht unterstuetzter Dateityp"):
                load_data(path)

    def test_txt_file_loading(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.txt"
            path.write_text(
                "# Telemetriedaten\n"
                "aa 00 1a 00 01 00 fa ff 19 20 2d ff fb fe 50 ff\n"
                "0x86, 0xc7, 0x6a, 0x00, aa, 00, 1a, 00 // line end\n",
                encoding="utf-8",
            )
            data = load_data(path)
            self.assertEqual(len(data), 24)
            self.assertEqual(data[:4], b"\xaa\x00\x1a\x00")



if __name__ == "__main__":
    unittest.main()
