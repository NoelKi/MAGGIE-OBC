import unittest

from lisa import calculate_crc32, verify_crc32


class CrcTests(unittest.TestCase):
    def test_mpeg2_reference_vector(self) -> None:
        self.assertEqual(calculate_crc32(b"123456789"), 0x0376E6E7)

    def test_verify_rejects_modified_data(self) -> None:
        expected = calculate_crc32(b"LISA")
        self.assertTrue(verify_crc32(b"LISA", expected))
        self.assertFalse(verify_crc32(b"lISA", expected))


if __name__ == "__main__":
    unittest.main()
