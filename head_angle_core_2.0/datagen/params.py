"""Toàn bộ tham số của bộ sinh dữ liệu. Mỗi nhóm ghi rõ NGUỒN. Dấu † = giả định của bản thảo, CHƯA có căn cứ đo.

Việc cần làm trước khi dùng dữ liệu để huấn luyện thật:
  - thay NoiseParams bằng kết quả Allan của MPU6050 với đúng cấu hình firmware (nhiệm vụ T2.2);
  - hiệu chỉnh StaParams theo số đo chênh lệch IMU - quang học trên người (nhiệm vụ T1.8 / T2.5).
"""
from dataclasses import dataclass, field, asdict
import json


@dataclass
class Physio:
    """Dải chuyển động tự nhiên của cổ (độ). † Giá trị cỡ ROM cổ người lớn khỏe mạnh theo y văn phổ biến;
    ĐỐI CHIẾU với ROM thực của nhóm nghiên cứu / tài liệu [4], [5], [15] rồi chỉnh. Dấu của pitch phụ thuộc quy ước gắn cảm biến."""
    pitch_deg: tuple = (-70.0, 65.0)     # †
    roll_deg: tuple = (-45.0, 45.0)      # †
    yaw_deg: tuple = (-80.0, 80.0)       # † (yaw chỉ dùng biên độ nhỏ, xem yaw_scale)
    roll_scale: float = 1.0              # † 1.0 = roll phủ toàn dải; hạ xuống nếu muốn roll nhỏ hơn
    yaw_scale: float = 0.15              # † yaw rất nhỏ vì không đo được và không có nhãn đáng tin
    v_peak_max_dps: float = 180.0        # † tốc độ đỉnh tối đa của chuyển động đầu so với thân; < ±250 °/s của firmware
    p_extreme: float = 0.25              # † xác suất chọn đích gần biên (85 đến 100% dải) để bao phủ toàn dải


@dataclass
class Motion:
    """Bộ sinh chuyển động tham số (bước 2): biên độ, tốc độ, thời gian dừng ngẫu nhiên."""
    static_s: float = 4.0                # đứng yên đầu phiên (hiệu chuẩn cần 3 s)
    v_peak_min_dps: float = 15.0         # † tốc độ đỉnh nhỏ nhất
    min_amp_deg: float = 3.0             # † bỏ chuyển động nhỏ hơn
    t_min_s: float = 0.3                 # † thời gian ngắn nhất của một đoạn
    pause_s: tuple = (0.2, 3.0)          # † thời gian dừng ngẫu nhiên
    p_long_pause: float = 0.1            # †
    long_pause_s: tuple = (3.0, 8.0)     # †
    breath_amp_deg: tuple = (0.2, 0.6)   # † nhịp thở làm thân dao động
    breath_hz: tuple = (0.2, 0.35)       # †
    sway_amp_deg: tuple = (0.3, 1.5)     # † dao động chậm của thân
    sway_hz: tuple = (0.05, 0.3)         # †
    p_bank: float = 0.5                  # xác suất lấy quỹ đạo từ ngân hàng quỹ đạo thật nếu có


@dataclass
class Sta:
    """Nhiễu mô mềm. Neo vào số đo da dịch chuyển tới 13,5 mm (gập, trục Z, C6) và 21 mm (nghiêng bên, trục X, C5) [9]."""
    d_max_pitch_mm: float = 13.5         # [9]
    d_max_roll_mm: float = 21.0          # [9]
    lever_head_m: float = 0.10           # † cánh tay đòn từ cảm biến EOP tới tâm quay
    lever_base_m: float = 0.12           # † cánh tay đòn từ cảm biến T1/T2 tới tâm quay
    tilt_coupling: float = 1.0           # hệ số nhân sau phép d/r; 1.0 = đúng công thức d/r (cận trên); có thể hạ xuống
    fc_hz: float = 1.0                   # † tần số cắt của quá trình STA chậm (lọc thông thấp)
    sigma0_mm: float = 0.1               # † STA lúc đứng yên (STA chậm 1 Hz với 0,5 mm đã làm gyro lúc nghỉ dao động quá mạnh)
    w_amp: float = 0.7                   # † trọng số biên độ trong độ lệch chuẩn STA
    w_vel: float = 0.3                   # † trọng số tốc độ
    v_ref_dps: float = 150.0             # † tốc độ chuẩn hóa
    sigma_over_dmax: float = 1.0 / 3.0   # độ lệch chuẩn tại biên = d_max / 3 (3 sigma ~ d_max)
    base_scale: float = 0.5              # † STA của cảm biến thân so với đầu
    lin_axes_scale: tuple = (1.0, 1.0, 0.3)   # † gia tốc tuyến tính giả theo ba trục cảm biến
    # nhóm nhân trắc: (khoảng độ dày mô mềm mm) † (tỉ lệ 0,7 : 1 : 1,27 bám MAE theo BMI của Christhudass 2025: 2,1 / 3,0 / 3,8)
    thickness_mm: dict = field(default_factory=lambda: {"low": (6.0, 8.0), "mid": (9.0, 11.0), "high": (11.5, 14.0)})
    thickness_ref_mm: float = 10.0       # †
    individual_logsd: float = 0.15       # † biến thiên giữa từng người


@dataclass
class Noise:
    """Nhiễu cảm biến. CÁC SỐ DƯỚI ĐÂY LÀ GIÁ TRỊ MẶC ĐỊNH CHỜ THAY bằng phân tích Allan thật (T2.2).
    Đơn vị sau bộ lọc DLPF, tại tần số xuất 50 Hz."""
    gyro_white_rads: float = 6e-4        # †
    accel_white_g: float = 2e-3          # †
    gyro_bias_sd_rads: float = 0.01      # † bias gyro mỗi cảm biến ~ N(0, sd)
    gyro_walk_rads_sqrts: float = 1e-5   # † bước ngẫu nhiên của bias
    accel_offset_sd_g: float = 0.02      # † độ lệch hiệu chuẩn gia tốc
    accel_scale_sd: float = 0.01         # † sai số tỷ lệ gia tốc
    mount_sd_deg: float = 3.0            # † lệch lắp đặt cảm biến so với xương
    dlpf_hz: float = 20.0                # DLPF_CFG = 4 trong .ino
    gyro_range_dps: float = 250.0        # GYRO_CONFIG = 0 trong .ino
    gyro_lsb_per_dps: float = 131.0
    accel_range_g: float = 2.0           # ACCEL_CONFIG = 0 trong .ino
    accel_lsb_per_g: float = 16384.0
    jitter_us: int = 300                 # † lệch timestamp
    drop_prob: float = 0.002             # † xác suất rơi một mẫu (sau 160 mẫu đầu)


@dataclass
class Params:
    fs_out: int = 50
    fs_sim: int = 200
    physio: Physio = field(default_factory=Physio)
    motion: Motion = field(default_factory=Motion)
    sta: Sta = field(default_factory=Sta)
    noise: Noise = field(default_factory=Noise)

    def to_json(self, path=None):
        s = json.dumps(asdict(self), indent=2, ensure_ascii=False)
        if path:
            open(path, "w", encoding="utf-8").write(s)
        return s

    @classmethod
    def from_dict(cls, d):
        p = cls()
        for k, v in d.items():
            cur = getattr(p, k)
            if hasattr(cur, "__dataclass_fields__"):
                for kk, vv in v.items():
                    setattr(cur, kk, tuple(vv) if isinstance(getattr(cur, kk), tuple) else vv)
            else:
                setattr(p, k, v)
        return p

    @classmethod
    def from_json(cls, path):
        return cls.from_dict(json.load(open(path, encoding="utf-8")))
