/* hap_madgwick.h - Madgwick thích nghi + ZUPT (AdaptiveMadgwick của Giai đoạn 1). */
#ifndef HAP_MADGWICK_H
#define HAP_MADGWICK_H
#include "hap_types.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    hap_quat_t q;
    float      beta;     /* beta của lần cập nhật gần nhất */
    bool       zupt;     /* gyro (đã trừ bias) nhỏ hơn ngưỡng ở lần cập nhật gần nhất */
} hap_madgwick_t;

void hap_madgwick_init(hap_madgwick_t *m, const hap_config_t *cfg);
void hap_madgwick_reset(hap_madgwick_t *m);
/* gx..gz: rad/s đã trừ bias; ax..az: g thô; dt: giây. Kết quả nằm ở m->q (đã chuẩn hóa). */
void hap_madgwick_update(hap_madgwick_t *m, const hap_config_t *cfg,
                         float gx, float gy, float gz, float ax, float ay, float az, float dt);

#ifdef __cplusplus
}
#endif
#endif
