/* hap_stream.h - luồng từng mẫu cho cả hệ thống: hiệu chuẩn -> warm-up -> chạy -> góc tương đối + LERP.
 *
 *   hap_stream_t s;  hap_stream_init(&s, &cfg);
 *   while (hap_stream_push_calib(&s, &x) == HAP_STATE_COLLECTING) { x = read(); }   // calib_samples mẫu
 *   while (!hap_stream_warmup_round(&s)) { }       // mỗi vòng ~ calib_samples x 2 lần cập nhật; có thể nhường CPU giữa các vòng
 *   hap_stream_step(&s, &x, &out);                 // từng mẫu, 50 Hz
 *   hap_stream_tare(&s);                           // phím Z của Giai đoạn 1
 */
#ifndef HAP_STREAM_H
#define HAP_STREAM_H
#include "hap_sensor.h"
#include "hap_lerp.h"
#include "hap_quat.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum { HAP_STATE_COLLECTING = 0, HAP_STATE_WARMUP = 1, HAP_STATE_RUNNING = 2 } hap_state_t;

typedef struct {
    float   bias1[3], bias2[3];   /* rad/s */
    float   std1[3],  std2[3];
    float   dq1, dq2;             /* delta Q của vòng warm-up cuối */
    uint8_t rounds;
    bool    converged;
    bool    vibration_warning;    /* std gyro > bias_std_warn trong lúc hiệu chuẩn */
} hap_calib_report_t;

typedef struct {
    float pitch_raw, roll_raw;        /* góc thô (độ), cái Giai đoạn 1 lưu vào CSV */
    float pitch_smooth, roll_smooth;  /* sau LERP nhân quả */
    hap_quat_t q1, q2;
    float beta1, beta2;
    bool  zupt1, zupt2;
    float yaw_raw;                    /* độ; KHÔNG đáng tin (không có từ kế), chỉ để log */
} hap_output_t;

typedef struct {
    hap_config_t       cfg;
    hap_state_t        state;
    hap_sensor_t       s1, s2;         /* s1 = T1-T3 (nền), s2 = đầu */
    hap_quat_t         qr1, qr2;       /* quaternion tham chiếu */
    hap_lerp_t         lp, lr;
    hap_sample_t       calib_buf[HAP_CALIB_MAX_SAMPLES];
    uint16_t           calib_n;
    hap_calib_report_t report;
} hap_stream_t;

void hap_stream_init(hap_stream_t *s, const hap_config_t *cfg);
hap_state_t hap_stream_push_calib(hap_stream_t *s, const hap_sample_t *x);
bool hap_stream_warmup_round(hap_stream_t *s);          /* true khi kết thúc (hội tụ hoặc hết vòng) */
bool hap_stream_step(hap_stream_t *s, const hap_sample_t *x, hap_output_t *out);  /* false nếu chưa RUNNING */
void hap_stream_tare(hap_stream_t *s);
const hap_calib_report_t *hap_stream_report(const hap_stream_t *s);

#ifdef __cplusplus
}
#endif
#endif
