"""Ghi dữ liệu thô đúng như pipeline Giai đoạn 1 đã xử lý, để phát lại và làm test hồi quy.

CÁCH GẮN VÀO head_angle_monitor_ble__final_fixed.py (3 dòng):

1) đầu tệp:     from raw_logger import RawLogger
                raw_logger = RawLogger("raw_session_001.csv")

2) trong calibrate(), ngay sau   samples.append(dq.get())   đổi thành:
                v = dq.get(); raw_logger.write(v); samples.append(v)

3) trong update(), ngay sau       v = dq.get(); done += 1     thêm:
                raw_logger.write(v)

4) khi đóng chương trình:  raw_logger.close()

Ghi ở hai điểm lấy khỏi hàng đợi (không ghi ở chỗ nhận BLE) vì _notify_handler có thể bỏ mẫu khi hàng đợi đầy
(self.dropped); chỉ những mẫu thật sự đi vào calibrate/update mới ảnh hưởng đầu ra. Mẫu hiệu chuẩn (150 mẫu đầu)
bắt buộc phải có, nếu không không phát lại được.

Định dạng (cùng định dạng đầu vào của golden.replay và c/host_test/replay):
    seq,ts_us,a1x,a1y,a1z,g1x,g1y,g1z,a2x,a2y,a2z,g2x,g2y,g2z
Số thực ghi bằng repr() nên khứ hồi chính xác với giá trị Python đã parse từ gói BLE.
"""


class RawLogger:
    HEADER = "seq,ts_us,a1x,a1y,a1z,g1x,g1y,g1z,a2x,a2y,a2z,g2x,g2y,g2z"

    def __init__(self, path):
        self.f = open(path, "w", buffering=1 << 16)
        self.f.write(self.HEADER + "\n")

    def write(self, v):
        """v: list 16 phần tử của validate_and_parse (bố cục mới): [ts, a1(3), g1(3), a2(3), g2(3), T1, T2, seq]."""
        vals = [str(int(v[15])), str(int(v[0]))] + [repr(float(x)) for x in v[1:13]]
        self.f.write(",".join(vals) + "\n")

    def close(self):
        self.f.close()
