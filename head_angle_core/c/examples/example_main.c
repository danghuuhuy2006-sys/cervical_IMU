/* example_main.c - MẪU tích hợp ESP-IDF cho ESP32-S3. CHƯA biên dịch và CHƯA chạy trên phần cứng.
 * Các hàm imu_read_two_sensors() và ble_publish() là chỗ giữ chỗ: nối với driver MPU6050 và BLE thật của bạn.
 *
 * Phân bổ lõi (DD-01): vòng thời gian thực ở lõi 1, BLE ở lõi 0. Dữ liệu đi qua hàng đợi FreeRTOS. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "hap_stream.h"

#define SAMPLE_PERIOD_US 20000   /* 50 Hz */

typedef struct { float pitch_raw, roll_raw, pitch_smooth, roll_smooth; uint32_t ts_us; uint8_t flags; } hap_msg_t;

static QueueHandle_t s_q;
static hap_stream_t  s_stream;   /* ~8 KB, đặt ở vùng tĩnh, không cấp phát động */

/* TODO: đọc 2 MPU6050 đúng như esp32_dual_mpu6050_ble_fix.ino (bản cuối của Giai đoạn 1):
 *   - cảm biến 1 (T13) địa chỉ 0x69, cảm biến 2 (đầu) địa chỉ 0x68, I2C 400 kHz
 *   - thanh ghi: SMPLRT_DIV = 0x09, CONFIG (DLPF) = 0x04, GYRO_CONFIG = 0x00 (±250 °/s), ACCEL_CONFIG = 0x00 (±2 g)
 *   - đổi đơn vị: accel = raw / 16384 (g); gyro = raw / 131 * 0.017453293 (rad/s)
 *   - chân SDA = 4, SCL = 5 là chân của bản ESP32-C3; phải kiểm tra lại cho board ESP32-S3.
 * Không đổi cấu hình DLPF/dải đo, vì tham số nhiễu và mô hình AI phụ thuộc vào chúng. */
extern bool imu_read_two_sensors(hap_sample_t *x);
/* TODO: gửi qua BLE (chế độ Live: gói nhỏ ~100 ms một lần). */
extern void ble_publish(const hap_msg_t *m);

static void rt_task(void *arg)
{
    (void)arg;
    hap_config_t cfg; hap_config_default(&cfg);
    hap_stream_init(&s_stream, &cfg);

    TickType_t last = xTaskGetTickCount();
    hap_sample_t x;

    /* 1) thu mẫu hiệu chuẩn tĩnh (bệnh nhân giữ yên 3 s) */
    while (s_stream.state == HAP_STATE_COLLECTING) {
        if (imu_read_two_sensors(&x)) { x.ts_us = (uint32_t)esp_timer_get_time(); (void)hap_stream_push_calib(&s_stream, &x); }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    }
    /* 2) warm-up: mỗi vòng ~ 300 lần cập nhật Madgwick; nhường CPU giữa các vòng, chưa đo thời gian thực tế */
    while (!hap_stream_warmup_round(&s_stream)) { vTaskDelay(1); }
    last = xTaskGetTickCount();

    /* 3) chạy từng mẫu */
    for (;;) {
        if (imu_read_two_sensors(&x)) {
            x.ts_us = (uint32_t)esp_timer_get_time();     /* uint32 tràn sau ~71,6 phút, lõi đã xử lý đúng */
            hap_output_t o;
            if (hap_stream_step(&s_stream, &x, &o)) {
                hap_msg_t m = { o.pitch_raw, o.roll_raw, o.pitch_smooth, o.roll_smooth, x.ts_us,
                                (uint8_t)((o.zupt1 ? 1u : 0u) | (o.zupt2 ? 2u : 0u)) };
                (void)xQueueSend(s_q, &m, 0);
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));        /* ghi chú: timer phần cứng tốt hơn (DD-S1) */
    }
}

static void ble_task(void *arg)
{
    (void)arg;
    hap_msg_t m;
    for (;;) { if (xQueueReceive(s_q, &m, portMAX_DELAY)) { ble_publish(&m); } }
}

void app_main(void)
{
    s_q = xQueueCreate(64, sizeof(hap_msg_t));
    xTaskCreatePinnedToCore(ble_task, "ble", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(rt_task, "rt", 8192, NULL, 10, NULL, 1);
}
