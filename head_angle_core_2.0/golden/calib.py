"""Hiệu chuẩn tĩnh: bias gyro (trung bình) + warm-up Madgwick tới hội tụ để lấy quaternion tham chiếu.

Tính bằng float32, cộng tuần tự (không dùng np.mean/np.std) để phép tính khớp từng bước với mã C.
Hành vi theo calibrate() của Giai đoạn 1:
  1. bias gyro = trung bình calib_samples mẫu đầu (mỗi cảm biến);
  2. reset Madgwick, lặp lại các mẫu hiệu chuẩn tối đa 15 vòng cho tới khi ΔQ < 0.0005 ở cả hai cảm biến;
  3. quaternion tham chiếu qr = quaternion của bộ lọc sau vòng cuối.
Bài báo mô tả bước 3 là "trung bình 50 quaternion đầu"; code thực tế dùng warm-up như trên.
"""
import numpy as np
from .config import F

_0 = F(0)


def mean_std3(vecs):
    """vecs: danh sách 3-tuple. Trả (mean[3], std[3]) float32, std tổng thể (ddof=0)."""
    n = F(len(vecs))
    mean, std = [], []
    for k in range(3):
        s = _0
        for v in vecs:
            s = s + F(v[k])
        m = s / n
        acc = _0
        for v in vecs:
            d = F(v[k]) - m
            acc = acc + d * d
        mean.append(m)
        std.append(np.sqrt(acc / n))
    return mean, std


def norm4_diff(q, p):
    d0, d1, d2, d3 = q[0] - p[0], q[1] - p[1], q[2] - p[2], q[3] - p[3]
    return np.sqrt(d0*d0 + d1*d1 + d2*d2 + d3*d3)
