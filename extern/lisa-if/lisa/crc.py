"""
CRC-32/MPEG-2 Prüfsummenberechnung gemäß LISA-ODISEE ICD v0.9 (Kapitel 3.3).

Spezifikation:
- Generatorpolynom : 0x04C11DB7 (x^32 + x^26 + x^23 + ...)
- Initialwert      : 0xFFFFFFFF
- RefIn / RefOut   : False (MSB-first)
- XorOut           : 0x00000000 (kein abschließendes XOR)
- Testvektor       : b"123456789" -> 0x0376E6E7
"""

POLYNOMIAL = 0x04C11DB7
INITIAL_CRC = 0xFFFFFFFF


def calculate_crc32(data: bytes) -> int:
    """Berechnet die 32-Bit MPEG-2 CRC-Prüfsumme für das übergebene Byte-Array."""
    crc = INITIAL_CRC
    for byte in data:
        crc ^= (byte << 24)
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ POLYNOMIAL) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


def verify_crc32(data: bytes, expected_crc: int) -> bool:
    """Gibt True zurück, wenn die berechnete Prüfsumme dem erwarteten Wert entspricht."""
    return calculate_crc32(data) == expected_crc
