"""SensorPipeline: trừ bias, cập nhật bias EMA khi ZUPT, tính dt từ timestamp, gọi Madgwick."""
import numpy as np
from .config import F
from .madgwick import AdaptiveMadgwick

_1E6 = F(1e-6)
_0 = F(0)


class SensorPipeline:
    def __init__(self, cfg):
        self.c = cfg
        self.madgwick = AdaptiveMadgwick(cfg)
        self.gyro_bias = [_0, _0, _0]
        self.last_ts_us = None        # int µs, modulo 2^32 như uint32 trên vi điều khiển
        self.zupt = False

    def process(self, gx, gy, gz, ax, ay, az, ts_us):
        c = self.c
        gx = F(gx) - self.gyro_bias[0]
        gy = F(gy) - self.gyro_bias[1]
        gz = F(gz) - self.gyro_bias[2]

        # Bias EMA chỉ chạy khi ZUPT (gyro đã trừ bias nhỏ hơn ngưỡng)
        gyro_mag = np.sqrt(gx*gx + gy*gy + gz*gz)
        self.zupt = bool(gyro_mag < c.zupt_thr)
        if self.zupt:
            a = c.bias_alpha
            self.gyro_bias[0] = self.gyro_bias[0] + a * gx
            self.gyro_bias[1] = self.gyro_bias[1] + a * gy
            self.gyro_bias[2] = self.gyro_bias[2] + a * gz

        dt = None
        ts = int(ts_us) & 0xFFFFFFFF
        if self.last_ts_us is not None:
            diff = (ts - self.last_ts_us) & 0xFFFFFFFF     # modulo như uint32
            raw_dt = F(diff) * _1E6
            if c.dt_min <= raw_dt <= c.dt_max:
                dt = raw_dt
        self.last_ts_us = ts
        return self.madgwick.update(gx, gy, gz, ax, ay, az, dt)

    def get_q(self):
        return self.madgwick.get_q()

    def reset(self):
        self.madgwick.reset()
        self.last_ts_us = None
