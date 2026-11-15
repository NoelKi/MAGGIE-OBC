from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import tempfile
import unittest

from lisa.cli import main


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class CliTests(unittest.TestCase):
    def test_sample_output_has_real_final_frame_numbers(self) -> None:
        stdout = StringIO()
        with redirect_stdout(stdout):
            exit_code = main([str(PROJECT_ROOT / "sample_data.json")])

        output = stdout.getvalue()
        self.assertEqual(exit_code, 0)
        self.assertIn("Gueltige Pakete   : 273", output)
        self.assertIn("CR rekonstruiert  : 51", output)
        self.assertIn(" 269 |", output)
        self.assertIn(" 273 |", output)

    def test_input_error_returns_nonzero_exit_code(self) -> None:
        stderr = StringIO()
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing.json"
            with redirect_stderr(stderr):
                exit_code = main([str(missing)])

        self.assertEqual(exit_code, 2)
        self.assertIn("Datei nicht gefunden", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
