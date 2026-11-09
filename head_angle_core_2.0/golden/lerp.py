"""Module LERP, tách riêng khỏi chuỗi dung hợp.

CausalLerp: bộ làm mượt nhân quả đúng như code Giai đoạn 1 (y += alpha * (x - y)), chạy từng mẫu.
lerp_interp: công thức 7 (nội suy tuyến tính giữa hai mốc), hàm thuần dùng cho đối chiếu chéo.
Phần tái lấy mẫu trên toàn chuỗi (không nhân quả) nằm ở golden/resample.py, chỉ dùng trên PC.
"""
from .config import F


class CausalLerp:
    def __init__(self, alpha, y0=0.0):
        self.alpha = F(alpha)
        self.y = F(y0)          # Giai đoạn 1 khởi tạo bằng 0, không phải mẫu đầu tiên

    def reset(self, y0=0.0):
        self.y = F(y0)

    def step(self, x):
        self.y = self.y + self.alpha * (F(x) - self.y)
        return self.y


def lerp_interp(t_star, t_i, t_ip1, v_i, v_ip1):
    t_star, t_i, t_ip1, v_i, v_ip1 = (F(t_star), F(t_i), F(t_ip1), F(v_i), F(v_ip1))
    return v_i + (t_star - t_i) / (t_ip1 - t_i) * (v_ip1 - v_i)
