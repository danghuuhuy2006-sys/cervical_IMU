# head_angle_core: golden model float32 và lõi C cho ESP32-S3

Chuyển từ bản cuối của Giai đoạn 1: `head_angle_monitor_ble__final_fixed.py` và `esp32_dual_mpu6050_ble_fix.ino`.
Đơn vị: gyro rad/s, gia tốc g, góc độ, thời gian µs.

## Cấu trúc
| Thư mục | Nội dung |
|---|---|
| `golden/` | Golden model Python, float32, từng mẫu, nhân quả. Mỗi tệp một khối: `madgwick`, `sensor` (bias EMA, ZUPT, dt), `calib`, `lerp` (nhân quả), `resample` (offline, chỉ PC), `quat`, `stream`, `protocol` (CRC8, định dạng gói) |
| `c/components/head_angle_core/` | Component ESP-IDF, C99, không cấp phát động, cùng cấu trúc module với `golden/` |
| `c/host_test/` | `make test` (kiểm thử đơn vị lõi C), `make replay` (phát lại CSV qua lõi C) |
| `reference/` | Nạp nguyên văn lớp/hàm của Giai đoạn 1 bằng `ast`, nhận diện được cả bản cũ lẫn bản cuối |
| `tests/` | `test_equivalence.py`, `test_edge_cases.py` |
| `vectors/` | Đầu vào và đầu ra chuẩn (20 s, mô phỏng) để kiểm firmware trên thiết bị |

## Chạy kiểm tra
    python3 tests/test_equivalence.py <head_angle_monitor_ble__final_fixed.py> 60 [<bản cũ ..._-final.py>]
    python3 tests/test_edge_cases.py  <head_angle_monitor_ble__final_fixed.py>
    make -C c/host_test test

## Kết quả đã kiểm (trên máy chủ, dữ liệu mô phỏng)
- Lõi thuật toán (AdaptiveMadgwick, SensorPipeline, q_mul, q_inv, q_to_euler, relative_pitch_roll, hằng số) của bản cuối GIỐNG bản cũ; kết quả hai bản trùng khớp tuyệt đối (lệch 0) trên cùng dữ liệu. Chỉ `calibrate` khác, do thứ tự trường trong mẫu đổi (ts lên đầu, seq xuống cuối) và bỏ cảnh báo rung.
- Mã C so với golden float32: lệch tối đa 7,7e-6° (pitch, roll, LERP, yaw).
- Golden float32 so với code Giai đoạn 1 (float64): tối đa 6,1e-4° trong 60 s, 4,6e-3° trong 10 phút.
- Kiểm thử biên (gia tốc 0, timestamp có khoảng trống/đi lùi, tràn uint32) đạt; test đơn vị lõi C đạt 11/11.

## Điểm khác giữa BÀI BÁO và CODE Giai đoạn 1 (mã chuyển đổi theo CODE)
| Mục | Bài báo | Code (python + .ino) |
|---|---|---|
| beta_min | 0,006 | 0,02 |
| Hệ số EMA của bias ZUPT | 0,01 | 2e-4 |
| Quaternion tham chiếu | trung bình 50 quaternion đầu | warm-up Madgwick lặp tới hội tụ (tối đa 15 vòng) |
| Dải gia tốc | ±4 g | ±2 g (ACCEL_CONFIG = 0x00, 1/16384) |
| Dải gyro | ±500 °/s | ±250 °/s (GYRO_CONFIG = 0x00, 1/131) |
| Lọc số của MPU6050 | không nêu | DLPF_CFG = 4 (khoảng 20 Hz), SMPLRT_DIV = 9, đọc 50 Hz |
| Hiệu chuẩn gia tốc 6 vị trí | có | KHÔNG có ở cả .py lẫn .ino (gia tốc chỉ nhân 1/16384) |
| LERP | làm mượt và tái lấy mẫu | y += alpha*(x - y), khởi tạo y = 0, chỉ cho hiển thị (bản cuối còn CSV lưu trung bình cộng dồn, không phải LERP) |

## Script Python bản cuối (`..._final_fixed.py`)
Biên dịch được, không còn tên biến chưa định nghĩa, và 31 cột tiêu đề CSV khớp 31 khóa ghi dòng. Bản trung gian `..._no-update.py` (có yaw, không chạy được) không còn cần dùng.
Hai thứ trong mã chuyển đổi KHÔNG có ở Giai đoạn 1, chỉ là bổ sung phụ: `yaw_raw` (bản cuối đã bỏ yaw; yaw trôi vì không có từ kế, chỉ để ghi log, có thể bỏ qua) và `vibration_warning` trong báo cáo hiệu chuẩn (bản cuối đã bỏ cảnh báo này).

## Về firmware .ino
- Timestamp là `esp_timer_get_time()` 64-bit. Lõi C dùng `uint32_t` (hiệu modulo 2^32) nên truyền `(uint32_t)esp_timer_get_time()`.
- `seq_num` chỉ tăng khi đọc I2C thành công, và vòng lặp tự bỏ nhịp khi trễ quá 2 chu kỳ. Hai trường hợp này không tạo khoảng trống seq nên số gói mất tính theo seq có thể thấp hơn thực tế; nên phân tích thêm khoảng trống timestamp.
- Chân I2C (SDA 4, SCL 5) là của ESP32-C3; kiểm tra lại cho ESP32-S3. Firmware dùng thư viện BLE Arduino và mã `.ino` chưa được chuyển sang ESP-IDF.

## Quy ước float32
- Mọi số thực là float; timestamp là uint32_t (float32 chỉ giữ chính xác tới khoảng 16,7 s tính theo µs). Hiệu hai timestamp lấy bằng số nguyên modulo 2^32 rồi mới đổi sang float, nên tràn sau khoảng 71,6 phút vẫn đúng.
- Biên dịch với `-ffp-contract=off`, không dùng `-ffast-math`; lõi qua `-Wdouble-promotion -Werror`.
- Không bit-exact giữa Python và C: sai khác còn lại do `asinf`/`atan2f` của thư viện toán.

## Hành vi giữ nguyên có chủ đích
Cập nhật q0..q3 tuần tự (q1 dùng q0 đã cập nhật, ...), khác công thức Madgwick chuẩn. Sửa thì phải tính lại số liệu Giai đoạn 1.

## Chưa làm
- Chưa biên dịch bằng ESP-IDF, chưa chạy trên ESP32-S3; `c/examples/example_main.c` chỉ là mẫu.
- Chưa đo thời gian, RAM stack, dòng tiêu thụ trên chip. `hap_stream_t` tĩnh khoảng 8 KB (sizeof trên máy chủ).
- Chưa có vector chuẩn từ dữ liệu thật. CSV của bản cuối có gia tốc và gyro thô nhưng không lưu 150 mẫu hiệu chuẩn, bias, qr1, qr2, và timestamp chỉ giữ 0,1 ms, nên không phát lại được. Cần ghi thêm các thứ đó.
