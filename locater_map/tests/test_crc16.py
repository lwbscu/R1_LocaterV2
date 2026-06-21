from locater_map.crc16 import crc16_ccitt_false, crc16_hex


def test_crc16_ccitt_false_known_vector():
    assert crc16_ccitt_false(b"123456789") == 0x29B1
    assert crc16_hex("123456789") == "29B1"
