from dataclasses import dataclass
from typing import Tuple

V3 = Tuple[float, float, float]


@dataclass
class Sample:
    """Một mẫu của cả hai cảm biến. Cảm biến 1 = T1-T3 (nền), cảm biến 2 = đầu."""
    ts_us: int            # số nguyên µs: float32 chỉ giữ chính xác tới ~16,7 s nên không dùng float
    a1: V3                # gia tốc (g)
    g1: V3                # gyro (rad/s)
    a2: V3
    g2: V3


@dataclass
class Output:
    pitch_raw: float
    roll_raw: float
    pitch_smooth: float
    roll_smooth: float
    q1: tuple
    q2: tuple
    beta1: float
    beta2: float
    zupt1: bool
    zupt2: bool
    yaw_raw: float = 0.0     # độ; KHÔNG đáng tin (không có từ kế), chỉ để log
