"""Bộ sinh dữ liệu huấn luyện EdgeAI (STA mô phỏng). Xem README_datagen.md.

Luồng: quỹ đạo đầu/thân sạch -> hướng xương -> hướng cảm biến (xương x lệch lắp đặt x STA(t)) -> IMU lý tưởng
-> bộ lọc DLPF + nhiễu cảm biến + lượng tử hóa -> chạy qua golden model lấy theta_base -> nhãn delta = theta_ref - theta_base.
"""
from .params import Params
from .dataset import make_dataset, simulate_sequence, sample_subject, loso_splits

__all__ = ["Params", "make_dataset", "simulate_sequence", "sample_subject", "loso_splits"]
