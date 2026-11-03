/* hap_types.h - kiểu dữ liệu và cấu hình dùng chung.
 * Đơn vị: gyro rad/s, gia tốc g, góc độ, thời gian micro-giây (uint32_t, modulo 2^32).
 * Toàn bộ số thực là float (32-bit). Không cấp phát động. */
#ifndef HAP_TYPES_H
#define HAP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAP_CALIB_MAX_SAMPLES
#define HAP_CALIB_MAX_SAMPLES 150u      /* = CALIB_SECONDS(3) * SAMPLE_RATE_HZ(50) */
#endif

typedef struct { float w, x, y, z; } hap_quat_t;

/* Một mẫu của cả hai cảm biến. Cảm biến 1 = T1-T3 (nền), cảm biến 2 = đầu. */
typedef struct {
    uint32_t ts_us;
    float a1[3], g1[3];
    float a2[3], g2[3];
} hap_sample_t;

/* Giá trị mặc định lấy từ head_angle_monitor_ble-_final.py (không phải từ bài báo). */
typedef struct {
    float    beta_min;          /* 0.02   (bài báo: 0.006) */
    float    beta_max;          /* 0.08 */
    float    accel_thr;         /* 0.08 g */
    float    zupt_thr;          /* 0.002 rad/s */
    float    bias_alpha;        /* 2e-4   (bài báo: 0.01) */
    float    nominal_dt;        /* 1/50 s */
    float    dt_min;            /* 0.005 s */
    float    dt_max;            /* 0.1 s */
    float    lerp_alpha;        /* 0.25 */
    uint16_t calib_samples;     /* 150, tối đa HAP_CALIB_MAX_SAMPLES */
    uint8_t  warmup_max_rounds; /* 15 */
    float    warmup_tol;        /* 0.0005 */
    float    bias_std_warn;     /* 0.005 rad/s */
} hap_config_t;

void hap_config_default(hap_config_t *cfg);

#ifdef __cplusplus
}
#endif
#endif /* HAP_TYPES_H */
