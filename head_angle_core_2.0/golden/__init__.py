"""Golden model (float32, streaming, nhân quả) của thuật toán Giai đoạn 1.

Đơn vị: gyro rad/s, gia tốc g, góc độ, thời gian µs (số nguyên).
Mọi phép tính số thực dùng numpy.float32; mã C trong c/components/head_angle_core
phải cho cùng kết quả (sai khác chỉ còn do asin/atan2 của thư viện toán).
"""
from .config import Config, F
from .types import Sample, Output
from .stream import HeadAngleStream
from .lerp import CausalLerp, lerp_interp

__all__ = ["Config", "F", "Sample", "Output", "HeadAngleStream", "CausalLerp", "lerp_interp"]
