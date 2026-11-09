"""Madgwick thích nghi + ZUPT, chép nguyên văn AdaptiveMadgwick (Giai đoạn 1) sang float32.

GIỮ NGUYÊN HÀNH VI: các dòng cập nhật q0..q3 chạy tuần tự, nên q1 dùng q0 ĐÃ cập nhật, q2 dùng
q0 và q1 đã cập nhật, v.v. Cách này khác công thức Madgwick chuẩn (dùng q cũ cho cả bốn thành phần).
Giữ nguyên để khớp kết quả Giai đoạn 1; sửa thì phải tính lại số liệu Giai đoạn 1.
"""
import numpy as np
from .config import F

_H, _1, _2, _4, _8, _0 = F(0.5), F(1), F(2), F(4), F(8), F(0)
_EPS = F(1e-6)


class AdaptiveMadgwick:
    def __init__(self, cfg):
        self.c = cfg
        self.q = (_1, _0, _0, _0)
        self.beta = cfg.beta_max
        self.zupt = False

    def reset(self):
        self.q = (_1, _0, _0, _0)

    def get_q(self):
        return self.q

    def update(self, gx, gy, gz, ax, ay, az, dt=None):
        c = self.c
        gx, gy, gz, ax, ay, az = F(gx), F(gy), F(gz), F(ax), F(ay), F(az)
        dt = c.nominal_dt if dt is None else F(dt)

        # beta thích nghi theo gia tốc thô (chưa chuẩn hóa)
        dev = np.abs(np.sqrt(ax*ax + ay*ay + az*az) - _1)
        alpha = dev / c.accel_thr
        if alpha > _1:
            alpha = _1
        self.beta = c.beta_min + (_1 - alpha) * (c.beta_max - c.beta_min)
        beta = self.beta
        q0, q1, q2, q3 = self.q

        # ZUPT: gyro (đã trừ bias) gần 0 thì bỏ tích phân gyro
        gyro_mag = np.sqrt(gx*gx + gy*gy + gz*gz)
        self.zupt = bool(gyro_mag < c.zupt_thr)
        if self.zupt:
            gx = gy = gz = _0

        n = np.sqrt(ax*ax + ay*ay + az*az)
        if n < _EPS:
            q0 += _H * (-q1*gx - q2*gy - q3*gz) * dt
            q1 += _H * ( q0*gx + q2*gz - q3*gy) * dt
            q2 += _H * ( q0*gy - q1*gz + q3*gx) * dt
            q3 += _H * ( q0*gz + q1*gy - q2*gx) * dt
        else:
            ax /= n; ay /= n; az /= n
            q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3
            s0 = _4*q0*q2q2 + _2*q2*ax + _4*q0*q1q1 - _2*q1*ay
            s1 = (_4*q1*q3q3 - _2*q3*ax + _4*q0q0*q1 - _2*q0*ay
                  - _4*q1 + _8*q1*q1q1 + _8*q1*q2q2 + _4*q1*az)
            s2 = (_4*q0q0*q2 + _2*q0*ax + _4*q2*q3q3 - _2*q3*ay
                  - _4*q2 + _8*q2*q1q1 + _8*q2*q2q2 + _4*q2*az)
            s3 = _4*q1q1*q3 - _2*q1*ax + _4*q2q2*q3 - _2*q2*ay
            sn = np.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
            if sn > _EPS:
                s0 /= sn; s1 /= sn; s2 /= sn; s3 /= sn
            q0 += (_H*(-q1*gx - q2*gy - q3*gz) - beta*s0) * dt
            q1 += (_H*( q0*gx + q2*gz - q3*gy) - beta*s1) * dt
            q2 += (_H*( q0*gy - q1*gz + q3*gx) - beta*s2) * dt
            q3 += (_H*( q0*gz + q1*gy - q2*gx) - beta*s3) * dt

        qn = np.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q = (F(q0/qn), F(q1/qn), F(q2/qn), F(q3/qn))
        return self.q
