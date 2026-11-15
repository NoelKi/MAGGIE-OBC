import math
import struct
import unittest

from lisa import Frame, MessageId
from lisa.payloads import (
    JointStatePayload,
    RawPayload,
    RcuImuPayload,
    TemperaturePayload,
    VoltagePayload,
)


class PayloadTests(unittest.TestCase):
    def test_rcu_imu_scaling(self) -> None:
        raw = struct.pack("<hhhhhh", 8192, -16384, 0, 3276, 0, -3276)
        frame = Frame(0, MessageId.RCU_IMU_DATA, raw, crc_valid=True)

        payload = frame.decoded_payload
        self.assertIsInstance(payload, RcuImuPayload)
        assert isinstance(payload, RcuImuPayload)
        self.assertEqual(payload.acc_g, (1.0, -2.0, 0.0))
        self.assertAlmostEqual(payload.acc_magnitude_g, math.sqrt(5.0))
        self.assertAlmostEqual(payload.gyro_dps[0], 49.98779296875)
        self.assertEqual(frame.acc_g, payload.acc_g)

    def test_non_imu_frame_has_no_fake_imu_values(self) -> None:
        frame = Frame(0, MessageId.VOLTAGE_DATA, struct.pack("<HH", 24000, 5000), True)
        self.assertIsNone(frame.acc_g)
        self.assertIsInstance(frame.decoded_payload, VoltagePayload)
        self.assertEqual(frame.details["bus_voltage_v"], 24.0)

    def test_joint_angle_is_signed_32_bit(self) -> None:
        # ICD v0.9: Payload 5 Byte (state + int32 Winkel, vorher 4 Byte int24).
        frame = Frame(0, MessageId.JOINT_STATE, b"\xc8\xff\xff\xff\xff", True)
        payload = frame.decoded_payload
        self.assertIsInstance(payload, JointStatePayload)
        assert isinstance(payload, JointStatePayload)
        self.assertEqual(payload.state, 200)
        self.assertEqual(payload.angle_urad, -1)

    def test_temperature_payload(self) -> None:
        # Skalierung /100, nicht /10 (siehe decode_temperature()-Docstring).
        frame = Frame(0, MessageId.TEMPERATURE_DATA, struct.pack("<hh", 253, -45), True)
        self.assertEqual(frame.decoded_payload, TemperaturePayload(2.53, -0.45))

    def test_undecoded_message_is_raw(self) -> None:
        raw = bytes(range(16))
        frame = Frame(0, MessageId.IMU_DATA, raw, True)
        self.assertEqual(frame.decoded_payload, RawPayload(raw))

    def test_known_message_rejects_wrong_payload_size(self) -> None:
        with self.assertRaisesRegex(ValueError, "erwartet 12"):
            Frame(0, MessageId.RCU_IMU_DATA, b"", True)


if __name__ == "__main__":
    unittest.main()
