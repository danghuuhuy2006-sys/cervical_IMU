"""Quaternion (w, x, y, z) float32. Khớp q_inv, q_mul, q_to_euler, relative_pitch_roll của Giai đoạn 1."""
import math
import numpy as np
from .config import F

RAD2DEG = F(180.0 / math.pi)
_ONE, _TWO = F(1), F(2)


def q_inv(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_mul(q, r):
    q0, q1, q2, q3 = q
    r0, r1, r2, r3 = r
    return (
        q0*r0 - q1*r1 - q2*r2 - q3*r3,
        q0*r1 + q1*r0 + q2*r3 - q3*r2,
        q0*r2 - q1*r3 + q2*r0 + q3*r1,
        q0*r3 + q1*r2 - q2*r1 + q3*r0,
    )


def q_to_euler(q):
    q0, q1, q2, q3 = q
    t = _TWO * (q0*q2 - q3*q1)
    if t > _ONE:
        t = _ONE
    if t < -_ONE:
        t = -_ONE
    pitch = np.arcsin(t) * RAD2DEG
    roll = np.arctan2(_TWO * (q0*q1 + q2*q3), _ONE - _TWO * (q1*q1 + q2*q2)) * RAD2DEG
    return F(pitch), F(roll)


def q_to_euler_full(q):
    """Như q_to_euler_full của bản Giai đoạn 1 cuối: (pitch, roll, yaw) độ.
    YAW KHÔNG ĐÁNG TIN: không có từ kế nên yaw trôi (xem mục hạn chế của bài báo). Chỉ để ghi log."""
    q0, q1, q2, q3 = q
    pitch, roll = q_to_euler(q)
    yaw = np.arctan2(_TWO * (q0*q3 + q1*q2), _ONE - _TWO * (q2*q2 + q3*q3)) * RAD2DEG
    return pitch, roll, F(yaw)


def relative_euler(q_base, q_head):
    q_rel = q_mul(q_inv(q_base), q_head)
    if q_rel[0] < 0:       # dạng chuẩn: phần thực dương
        q_rel = (-q_rel[0], -q_rel[1], -q_rel[2], -q_rel[3])
    return q_to_euler_full(q_rel)


def relative_pitch_roll(q_base, q_head):
    p, r, _ = relative_euler(q_base, q_head)
    return p, r
