import os
import tempfile
import unittest

import matplotlib
matplotlib.use("Agg")  # Headless Backend fuer Tests

from pathlib import Path

from lisa import (
    Frame,
    MessageId,
    extract_imu_series,
    load_data,
    parse_stream_detailed,
    plot_imu_data,
)
from lisa.cli import main as cli_main
from lisa.payloads import RcuImuPayload

SAMPLE_DATA_PATH = Path(__file__).resolve().parents[1] / "sample_data.json"



class PlotterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.raw_bytes = load_data(SAMPLE_DATA_PATH)
        self.parse_result = parse_stream_detailed(self.raw_bytes)

    def test_extract_imu_series(self) -> None:
        series = extract_imu_series(self.parse_result)
        self.assertEqual(len(series["indices"]), 273)
        self.assertEqual(len(series["acc_x"]), 273)
        self.assertEqual(len(series["gyro_x"]), 273)

        # Alternative Einheiten testen
        series_ms2 = extract_imu_series(self.parse_result, acc_unit="ms2", gyro_unit="rads")
        self.assertAlmostEqual(series_ms2["acc_x"][0], series["acc_x"][0] * 9.80665, places=4)

        # Datasheet scaling mode
        series_ds = extract_imu_series(self.parse_result, gyro_scale_mode="datasheet")
        self.assertNotEqual(series["gyro_x"][0], series_ds["gyro_x"][0])

    def test_plot_imu_data_saves_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out_file = os.path.join(tmpdir, "test_imu_plot.png")
            fig = plot_imu_data(self.parse_result, save_path=out_file, show=False)
            self.assertIsNotNone(fig)
            self.assertTrue(os.path.exists(out_file))
            self.assertGreater(os.path.getsize(out_file), 1000)

    def test_plot_imu_data_raises_on_empty_imu(self) -> None:
        empty_result = parse_stream_detailed(b"")
        with self.assertRaisesRegex(ValueError, "Keine RCU-IMU"):
            plot_imu_data(empty_result)

    def test_cli_save_plot(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            out_file = os.path.join(tmpdir, "cli_plot.png")
            exit_code = cli_main(["--save-plot", out_file, str(SAMPLE_DATA_PATH)])
            self.assertEqual(exit_code, 0)
            self.assertTrue(os.path.exists(out_file))


if __name__ == "__main__":
    unittest.main()
