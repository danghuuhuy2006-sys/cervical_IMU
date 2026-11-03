"""Sinh luồng IMU thô mô phỏng (2 cảm biến) để thử tương đương giữa các bản cài đặt.

Không phải dữ liệu thật và không dùng để đánh giá độ chính xác; chỉ để kiểm tra mã.
Gồm: 3 s tĩnh (hiệu chuẩn), chuyển động gập/ngửa + nghiêng của đầu, thân dao động nhẹ, bias gyro,
nhiễu trắng, vài đoạn gia tốc nhiễu (làm beta thay đổi), jitter timestamp và 2 gói bị mất.
Ghi CSV với 9 chữ số có nghĩa (khứ hồi chính xác cho float32).
"""
import sys
import numpy as np

FS = 50
DT = 1.0 / FS


def qmul(a, b):
    a0, a1, a2, a3 = a; b0, b1, b2, b3 = b
    return np.array([a0*b0 - a1*b1 - a2*b2 - a3*b3, a0*b1 + a1*b0 + a2*b3 - a3*b2,
                     a0*b2 - a1*b3 + a2*b0 + a3*b1, a0*b3 + a1*b2 - a2*b1 + a3*b0])


def qeuler(roll, pitch, yaw):
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


def gravity_in_body(q):
    q0, q1, q2, q3 = q
    return np.array([2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3])


def body_rate(qs):
    w = np.zeros((len(qs), 3))
    for k in range(len(qs) - 1):
        dq = qmul(np.array([qs[k][0], -qs[k][1], -qs[k][2], -qs[k][3]]), qs[k+1])
        if dq[0] < 0:
            dq = -dq
        w[k] = 2.0 * dq[1:] / DT
    w[-1] = w[-2]
    return w


def make(duration_s=60.0, seed=7):
    rng = np.random.default_rng(seed)
    n = int(duration_s * FS)
    t = np.arange(n) * DT
    still = t < 3.5                                   # 3 s hiệu chuẩn + dư
    ramp = np.clip((t - 3.5) / 1.5, 0, 1)

    d2r = np.pi / 180
    head_pitch = ramp * (25*np.sin(2*np.pi*0.18*t) + 10*np.sin(2*np.pi*0.41*t + 1.0)) * d2r
    head_roll = ramp * (8*np.sin(2*np.pi*0.13*t + 0.5)) * d2r
    head_yaw = ramp * (6*np.sin(2*np.pi*0.09*t)) * d2r
    base_pitch = ramp * (1.5*np.sin(2*np.pi*0.25*t)) * d2r
    base_roll = ramp * (1.0*np.sin(2*np.pi*0.31*t + 2.0)) * d2r

    q_head = np.array([qeuler(head_roll[k] + base_roll[k], head_pitch[k] + base_pitch[k], head_yaw[k]) for k in range(n)])
    q_base = np.array([qeuler(base_roll[k], base_pitch[k], 0.0) for k in range(n)])

    a1, a2 = np.array([gravity_in_body(q) for q in q_base]), np.array([gravity_in_body(q) for q in q_head])
    g1, g2 = body_rate(q_base), body_rate(q_head)

    # nhiễu gia tốc động ở vài đoạn (làm beta thay đổi)
    for (s, e, amp) in [(12.0, 12.8, 0.25), (30.0, 30.6, 0.4), (48.0, 49.0, 0.15)]:
        i0, i1 = int(s*FS), min(int(e*FS), n)
        if i0 >= n:
            continue
        a2[i0:i1] += amp * rng.standard_normal((i1 - i0, 3))

    bias1, bias2 = np.array([0.012, -0.008, 0.005]), np.array([-0.006, 0.010, 0.0035])
    g1 = g1 + bias1 + 6e-4 * rng.standard_normal((n, 3))
    g2 = g2 + bias2 + 6e-4 * rng.standard_normal((n, 3))
    a1 = a1 + 2e-3 * rng.standard_normal((n, 3))
    a2 = a2 + 2e-3 * rng.standard_normal((n, 3))

    ts = 1_234_567 + np.round(np.arange(n) * 1e6 * DT + rng.integers(-300, 301, n)).astype(np.int64)
    ts = np.maximum.accumulate(ts)
    keep = np.ones(n, bool)
    for k in (900, 1700):                             # hai gói bị mất trong lúc chuyển động
        if k < n:
            keep[k] = False
    seq = np.arange(n)
    cols = np.column_stack([seq, ts, a1, g1, a2, g2])[keep]
    return cols


def write(path, cols):
    names = "seq,ts_us,a1x,a1y,a1z,g1x,g1y,g1z,a2x,a2y,a2z,g2x,g2y,g2z"
    with open(path, "w") as f:
        f.write(names + "\n")
        for r in cols:
            vals = [str(int(r[0])), str(int(r[1]))] + ["%.9g" % np.float32(v) for v in r[2:]]
            f.write(",".join(vals) + "\n")


if __name__ == "__main__":
    out = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    write(out, make(dur))
