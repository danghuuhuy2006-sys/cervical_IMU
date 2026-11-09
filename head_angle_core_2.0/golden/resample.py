"""Tái lấy mẫu offline cho đối chiếu chéo với hệ tham chiếu (Kinovea/Brainlab). CHỈ DÙNG TRÊN PC.

Đây là nội suy KHÔNG nhân quả (cần mẫu tương lai), không được đưa vào chuỗi thời gian thực.
Dùng float64 vì trục thời gian của hệ tham chiếu là giây, không phải µs nguyên.
"""
import numpy as np


def resample_linear(t_src, v_src, t_dst):
    """Nội suy tuyến tính (công thức 7). t_src tăng dần; ngoài miền thì trả NaN."""
    t_src = np.asarray(t_src, dtype=np.float64)
    v_src = np.asarray(v_src, dtype=np.float64)
    t_dst = np.asarray(t_dst, dtype=np.float64)
    out = np.interp(t_dst, t_src, v_src)
    out[(t_dst < t_src[0]) | (t_dst > t_src[-1])] = np.nan
    return out


def align_and_error(t_imu, v_imu, t_ref, v_ref, t_offset=0.0):
    """Đưa góc IMU về trục thời gian của tham chiếu, trả (sai lệch, RMSE)."""
    v = resample_linear(np.asarray(t_imu) + t_offset, v_imu, t_ref)
    ok = ~np.isnan(v)
    err = v[ok] - np.asarray(v_ref)[ok]
    return err, float(np.sqrt(np.mean(err ** 2)))
