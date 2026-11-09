/* hap_quat.h - tiện ích quaternion, khớp q_inv/q_mul/q_to_euler/relative_pitch_roll của Giai đoạn 1. */
#ifndef HAP_QUAT_H
#define HAP_QUAT_H
#include "hap_types.h"
#ifdef __cplusplus
extern "C" {
#endif

#define HAP_RAD2DEG 57.29577951308232f

hap_quat_t hap_q_identity(void);
hap_quat_t hap_q_inv(hap_quat_t q);
hap_quat_t hap_q_mul(hap_quat_t q, hap_quat_t r);
void hap_q_to_euler(hap_quat_t q, float *pitch_deg, float *roll_deg);
/* Như q_to_euler_full của bản cuối: có thêm yaw. YAW KHÔNG ĐÁNG TIN (không có từ kế), chỉ để ghi log. */
void hap_q_to_euler_full(hap_quat_t q, float *pitch_deg, float *roll_deg, float *yaw_deg);
void hap_relative_euler(hap_quat_t q_base, hap_quat_t q_head, float *pitch_deg, float *roll_deg, float *yaw_deg);
void hap_relative_pitch_roll(hap_quat_t q_base, hap_quat_t q_head, float *pitch_deg, float *roll_deg);
float hap_q_dist(hap_quat_t a, hap_quat_t b);   /* chuẩn Euclid 4 chiều của (a - b) */

#ifdef __cplusplus
}
#endif
#endif
