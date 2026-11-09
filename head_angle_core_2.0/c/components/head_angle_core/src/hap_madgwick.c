/* Madgwick thích nghi + ZUPT, chuyển nguyên văn từ AdaptiveMadgwick (Giai đoạn 1) sang float.
 *
 * GIỮ NGUYÊN HÀNH VI: q0..q3 được cập nhật tuần tự, nên q1 dùng q0 ĐÃ cập nhật, q2 dùng q0 và q1
 * đã cập nhật, v.v. Khác công thức Madgwick chuẩn (dùng q cũ cho cả bốn thành phần). Không "sửa"
 * nếu chưa tính lại số liệu Giai đoạn 1. Biên dịch với -ffp-contract=off để không bị gộp FMA. */
#include "hap_madgwick.h"
#include <math.h>

void hap_madgwick_reset(hap_madgwick_t *m)
{
    m->q.w = 1.0f; m->q.x = 0.0f; m->q.y = 0.0f; m->q.z = 0.0f;
}

void hap_madgwick_init(hap_madgwick_t *m, const hap_config_t *cfg)
{
    hap_madgwick_reset(m);
    m->beta = cfg->beta_max;
    m->zupt = false;
}

void hap_madgwick_update(hap_madgwick_t *m, const hap_config_t *c,
                         float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    /* beta thích nghi theo gia tốc thô (chưa chuẩn hóa) */
    float dev   = fabsf(sqrtf(ax*ax + ay*ay + az*az) - 1.0f);
    float alpha = dev / c->accel_thr;
    if (alpha > 1.0f) { alpha = 1.0f; }
    m->beta = c->beta_min + (1.0f - alpha) * (c->beta_max - c->beta_min);
    float beta = m->beta;

    float q0 = m->q.w, q1 = m->q.x, q2 = m->q.y, q3 = m->q.z;

    /* ZUPT: gyro (đã trừ bias) gần 0 thì bỏ tích phân gyro */
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    m->zupt = (gyro_mag < c->zupt_thr);
    if (m->zupt) { gx = 0.0f; gy = 0.0f; gz = 0.0f; }

    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-6f) {
        q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
        q1 += 0.5f * ( q0*gx + q2*gz - q3*gy) * dt;
        q2 += 0.5f * ( q0*gy - q1*gz + q3*gx) * dt;
        q3 += 0.5f * ( q0*gz + q1*gy - q2*gx) * dt;
    } else {
        ax /= n; ay /= n; az /= n;
        float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
        float s0 = 4.0f*q0*q2q2 + 2.0f*q2*ax + 4.0f*q0*q1q1 - 2.0f*q1*ay;
        float s1 = (4.0f*q1*q3q3 - 2.0f*q3*ax + 4.0f*q0q0*q1 - 2.0f*q0*ay
                    - 4.0f*q1 + 8.0f*q1*q1q1 + 8.0f*q1*q2q2 + 4.0f*q1*az);
        float s2 = (4.0f*q0q0*q2 + 2.0f*q0*ax + 4.0f*q2*q3q3 - 2.0f*q3*ay
                    - 4.0f*q2 + 8.0f*q2*q1q1 + 8.0f*q2*q2q2 + 4.0f*q2*az);
        float s3 = 4.0f*q1q1*q3 - 2.0f*q1*ax + 4.0f*q2q2*q3 - 2.0f*q2*ay;
        float sn = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (sn > 1e-6f) { s0 /= sn; s1 /= sn; s2 /= sn; s3 /= sn; }
        q0 += (0.5f*(-q1*gx - q2*gy - q3*gz) - beta*s0) * dt;
        q1 += (0.5f*( q0*gx + q2*gz - q3*gy) - beta*s1) * dt;
        q2 += (0.5f*( q0*gy - q1*gz + q3*gx) - beta*s2) * dt;
        q3 += (0.5f*( q0*gz + q1*gy - q2*gx) - beta*s3) * dt;
    }

    float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    m->q.w = q0 / qn; m->q.x = q1 / qn; m->q.y = q2 / qn; m->q.z = q3 / qn;
}
