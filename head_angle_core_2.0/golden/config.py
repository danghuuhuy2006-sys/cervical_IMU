"""Hằng số cấu hình. Giá trị mặc định lấy từ head_angle_monitor_ble-_final.py (KHÔNG phải từ bài báo)."""
import os
from dataclasses import dataclass
import numpy as np

# Mặc định float32 (đúng với firmware). HAP_DTYPE=float64 chỉ để chạy "hồi quy logic" so với Giai đoạn 1 (float64):
# nếu cấu trúc lại không đổi logic thì chế độ này phải ra cùng số với code gốc (sai khác cỡ 1e-13 độ).
F = np.float64 if os.environ.get("HAP_DTYPE", "float32") == "float64" else np.float32


@dataclass
class Config:
    beta_min: float = 0.02        # BETA_MIN trong code (bài báo ghi 0.006)
    beta_max: float = 0.08        # BETA_MAX
    accel_thr: float = 0.08       # ACCEL_THR (g)
    zupt_thr: float = 0.002       # ZERO_VEL_THR (rad/s)
    bias_alpha: float = 2e-4      # SensorPipeline.BIAS_ALPHA (bài báo ghi 0.01)
    dt_min: float = 0.005
    dt_max: float = 0.1
    lerp_alpha: float = 0.25      # LERP_ALPHA
    calib_samples: int = 150      # CALIB_SECONDS * SAMPLE_RATE_HZ = 3 * 50
    warmup_max_rounds: int = 15   # MAX_ROUNDS
    warmup_tol: float = 0.0005    # CONVERGE_TOL
    bias_std_warn: float = 0.005  # BIAS_STD_WARN (rad/s)

    def __post_init__(self):
        for name in ("beta_min", "beta_max", "accel_thr", "zupt_thr", "bias_alpha",
                     "dt_min", "dt_max", "lerp_alpha", "warmup_tol", "bias_std_warn"):
            setattr(self, name, F(getattr(self, name)))
        self.nominal_dt = F(1.0) / F(50.0)      # = 1.0f/50.0f trong C
