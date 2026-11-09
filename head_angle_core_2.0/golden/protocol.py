"""Định dạng gói của Giai đoạn 1 (CSV 17 cột + CRC8). Chỉ dùng trên PC (phát lại log, đối chiếu)."""
from .types import Sample


def crc8(data: str) -> int:           # nguyên văn Giai đoạn 1
    crc = 0x00
    for b in data.encode("ascii"):
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc


def validate_and_parse(line: str):    # nguyên văn Giai đoạn 1: list 16 số thực hoặc None
    parts = line.split(",")
    if len(parts) != 17:
        return None
    try:
        payload = ",".join(parts[:16])
        expected_crc = int(parts[16])
        if expected_crc != crc8(payload):
            return None
        return list(map(float, parts[:16]))
    except (ValueError, IndexError):
        return None


def to_sample(v) -> Sample:
    """v: 16 giá trị [seq, ts_us, a1(3), g1(3), a2(3), g2(3), T1, T2]. Cảm biến 1 = T1-T3, 2 = đầu."""
    return Sample(ts_us=int(v[1]), a1=tuple(v[2:5]), g1=tuple(v[5:8]),
                  a2=tuple(v[8:11]), g2=tuple(v[11:14]))
