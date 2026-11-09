"""Quaternion float64 dạng vector hóa (w, x, y, z). Quy ước khớp golden/quat.py (ZYX, pitch quanh y, roll quanh x)."""
import numpy as np


def qmul(a, b):
    a0, a1, a2, a3 = np.moveaxis(a, -1, 0)
    b0, b1, b2, b3 = np.moveaxis(b, -1, 0)
    return np.stack([a0*b0 - a1*b1 - a2*b2 - a3*b3, a0*b1 + a1*b0 + a2*b3 - a3*b2,
                     a0*b2 - a1*b3 + a2*b0 + a3*b1, a0*b3 + a1*b2 - a2*b1 + a3*b0], axis=-1)


def qconj(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def qnorm(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def from_euler(roll, pitch, yaw):
    """Góc (rad) -> quaternion. Cùng công thức với tools/gen_synthetic.qeuler."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.stack([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy], axis=-1)


def from_rotvec(phi):
    """Vector quay (rad), (...,3) -> quaternion."""
    ang = np.linalg.norm(phi, axis=-1, keepdims=True)
    half = ang / 2
    small = ang < 1e-12
    axis = np.where(small, 0.0, phi / np.where(small, 1.0, ang))
    return np.concatenate([np.cos(half), axis * np.sin(half)], axis=-1)


def to_rotvec(q):
    """Quaternion -> vector quay (rad), chọn nhánh w >= 0."""
    q = np.where(q[..., :1] < 0, -q, q)
    v = q[..., 1:]
    s = np.linalg.norm(v, axis=-1, keepdims=True)
    ang = 2 * np.arctan2(s, q[..., :1])
    scale = np.where(s < 1e-12, 2.0, ang / np.where(s < 1e-12, 1.0, s))
    return v * scale


def gravity_in_body(q):
    """Hướng 1 g trong hệ cảm biến (đơn vị g), khớp công thức hàm mục tiêu của Madgwick trong Giai đoạn 1."""
    q0, q1, q2, q3 = np.moveaxis(q, -1, 0)
    return np.stack([2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3], axis=-1)


def body_rate(q, dt):
    """Vận tốc góc trong hệ cảm biến (rad/s) từ chuỗi quaternion lấy mẫu đều dt."""
    dq = qmul(qconj(q[:-1]), q[1:])
    w = to_rotvec(dq) / dt
    return np.concatenate([w, w[-1:]], axis=0)


def relative_pitch_roll_deg(q_base, q_head):
    """Giống golden.quat.relative_pitch_roll nhưng float64, vector hóa. Trả (pitch, roll) độ."""
    qr = qmul(qconj(q_base), q_head)
    qr = np.where(qr[..., :1] < 0, -qr, qr)
    w, x, y, z = np.moveaxis(qr, -1, 0)
    pitch = np.arcsin(np.clip(2 * (w*y - z*x), -1.0, 1.0))
    roll = np.arctan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    return np.degrees(pitch), np.degrees(roll)
