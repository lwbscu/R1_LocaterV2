from __future__ import annotations


def crc16_ccitt_false(data: bytes | str) -> int:
    """CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, xorout 0."""
    if isinstance(data, str):
        data = data.encode("ascii", errors="ignore")
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def crc16_hex(data: bytes | str) -> str:
    return f"{crc16_ccitt_false(data):04X}"
