# Bộ sinh dữ liệu huấn luyện EdgeAI (datagen)

`tools/gen_synthetic.py` KHÔNG phải bộ sinh này: nó chỉ tạo luồng thô đơn giản để kiểm thử mã (không có STA, không nhãn).
Bộ sinh thật nằm ở thư mục `datagen/`.

    python -m datagen.make --out data/syn --subjects 9 --seq 2 --seconds 120 --seed 0 [--backend c] [--bank dir] [--paired]

## Ánh xạ với các bước yêu cầu
| Yêu cầu | Cài đặt |
|---|---|
| 1. Tập hợp dữ liệu nền tảng (GĐ1, giàn thử, in-vivo) | `trajectory.TrajectoryBank`: đọc thư mục CSV (`t_s,pitch_deg,roll_deg`); quỹ đạo thật được co giãn biên độ (0,7 đến 1,1), co giãn thời gian (0,7 đến 1,4), chèn dừng, rồi kẹp theo dải sinh lý. Chưa có tệp thật nào nên ngân hàng đang rỗng (chỉ được thử bằng tệp giả trong test) |
| 2. Bộ sinh chuyển động tham số | `gen_axis_track`: chuỗi đoạn min-jerk tới đích ngẫu nhiên, tốc độ đỉnh lấy log-đều trong [15, 180] °/s, thời gian dừng ngẫu nhiên 0,2 đến 3 s (10% dừng dài 3 đến 8 s); thân có nhịp thở và dao động chậm |
| 3. Ràng buộc dải sinh lý | Đích được kẹp trong dải (`Physio`), 25% đích chọn ở 85 đến 100% biên để bao phủ toàn dải; tốc độ đỉnh <= 180 °/s (< ±250 °/s của firmware). Test: 0 vi phạm, bao phủ >= 90% mỗi phía |
| 4. Tập lệnh tự động | `datagen/make.py` (dòng lệnh) và `make_dataset()` |
| IMU lý tưởng | Gyro = vận tốc góc suy ra từ chuỗi quaternion cảm biến (200 Hz); gia tốc = trọng lực xoay vào hệ cảm biến |
| Phép quay STA | Hướng cảm biến = hướng xương x lệch lắp đặt x STA(t). STA(t) = nhiễu Gauss qua lọc thông thấp bậc 2 (1 Hz), độ lệch chuẩn tăng theo biên độ và tốc độ |
| Gia tốc tuyến tính giả | Dịch chuyển da p(t) (cùng loại quá trình), a = d²p/dt² cộng vào gia tốc |
| Neo vào số đo thực | d_max = 13,5 mm (gập) và 21 mm (nghiêng bên) [9]; sigma tại biên = d_max / 3; quy đổi mm sang độ bằng d / r (`sta_angle_deg`), r đầu 0,10 m và r thân 0,12 m là giả định † |
| Nhiễu cảm biến | Nhiễu trắng, bias + bước ngẫu nhiên, lệch/ tỷ lệ gia tốc, jitter timestamp, mẫu rơi; DLPF 20 Hz, kẹp ±250 °/s và ±2 g, lượng tử hóa 131 LSB/(°/s), 16384 LSB/g (đúng cấu hình `.ino`) |
| Đa dạng hóa | Lệch lắp đặt ngẫu nhiên (3° SD †), bias/offset/tỷ lệ theo từng người, nhóm BMI (độ dày mô mềm 6 đến 8 / 9 đến 11 / 11,5 đến 14 mm; tỉ lệ 0,70 : 1 : 1,27 bám MAE theo BMI của Christhudass 2025) và biến thiên cá thể |
| Nhãn | Dữ liệu IMU chạy qua golden (float32, hoặc mã C bằng `--backend c`) để có `theta_base`. Nhãn `delta = theta_ref - theta_base` |

## Quy ước nhãn quan trọng
- `theta_ref` = góc của cảm biến GẮN CỨNG (có lệch lắp đặt, không STA): đây là "tín hiệu sạch" kiểu Giai đoạn 1, nên nhãn tách riêng STA và nhiễu cảm biến. Nếu dùng góc giải phẫu làm tham chiếu thì lệch lắp đặt 3° đã đẩy RMS nhãn lên cỡ 4 độ (nhiễu chéo trục), che mất STA.
- `theta_anat` = góc xương trong hệ giải phẫu (không lệch lắp đặt, không STA), chỉ để phân tích.
- Dải sinh lý áp cho quỹ đạo xương (`rel_pitch_true`, `rel_roll_true`, `theta_anat`). `theta_ref`/`theta_base` có thể vượt dải vài độ (ví dụ roll tới 60° khi pitch gần ±70°) vì lệch lắp đặt và nhiễu chéo kiểu Euler; đây là hành vi đúng của cảm biến thật.
- Nhãn chỉ hợp lệ từ mẫu thứ 150 (150 mẫu đầu dùng hiệu chuẩn, `valid = False`). Delta gồm cả sai số động học của bộ lọc Madgwick, không chỉ STA; cặp `--paired` (`theta_base_noSTA`) cho phép tách phần STA.

## Tệp đầu ra
`<người>_<phiên>.npz`: `ts_us`, `a1,g1` (thân), `a2,g2` (đầu), `theta_ref`, `theta_anat`, `theta_base`, `delta`, `valid`, `sta_head_deg`, `sta_base_deg` (nhãn chẩn đoán, KHÔNG đưa vào đầu vào mô hình), `rel_pitch_true`, `rel_roll_true`, `meta` (JSON), thêm `theta_base_noSTA` nếu `--paired`.
Kèm `manifest.csv` (người, nhóm BMI, độ dày, hệ số STA), `params.json`, `dataset_meta.json`. Chia tập: `loso_splits(manifest)` (leave-one-subject-out theo người, không rò rỉ).

## Giới hạn: chưa đủ để huấn luyện kết quả thật
1. Mọi tham số có dấu † (nhiễu cảm biến, cánh tay đòn, tần số cắt STA, dải ROM, độ lệch lắp đặt) là giả định. Cần thay nhiễu bằng kết quả Allan của MPU6050 với đúng cấu hình `.ino` (T2.2) và hiệu chỉnh STA theo số đo chênh lệch IMU - quang học trên người (T1.8, T2.5).
2. Quy đổi d / r với `tilt_coupling = 1` là một cận trên (da trượt d chưa chắc làm cảm biến nghiêng d / r). Với mặc định, nhóm BMI giữa có RMS delta cỡ 2,7° so với 0,9° khi tắt STA; nếu số đo thật nhỏ hơn thì hạ `tilt_coupling`.
3. Phổ tần số STA (1 Hz) và quan hệ độ lệch chuẩn - biên độ - tốc độ là mô hình đơn giản, chưa kiểm chứng.
4. Dải ROM chỉ là ranh giới từng trục; chưa mô hình hóa ràng buộc kết hợp pitch - roll - yaw.
5. Cần so sánh phân phối chênh lệch của dữ liệu tổng hợp với dữ liệu người thật (nhiệm vụ T2.5) trước khi tin vào kết quả huấn luyện trên dữ liệu này.
