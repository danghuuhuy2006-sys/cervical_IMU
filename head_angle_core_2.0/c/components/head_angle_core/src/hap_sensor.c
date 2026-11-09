#include "hap_sensor.h"
#include <math.h>

void hap_sensor_init(hap_sensor_t *s, const hap_config_t *cfg)
{
    hap_madgwick_init(&s->mad, cfg);
    s->gyro_bias[0] = 0.0f; s->gyro_bias[1] = 0.0f; s->gyro_bias[2] = 0.0f;
    s->last_ts_us = 0u;
    s->has_ts = false;
}

void hap_sensor_reset(hap_sensor_t *s)
{
    hap_madgwick_reset(&s->mad);
    s->has_ts = false;
}

hap_quat_t hap_sensor_process(hap_sensor_t *s, const hap_config_t *c,
                              const float g[3], const float a[3], uint32_t ts_us)
{
    float gx = g[0] - s->gyro_bias[0];
    float gy = g[1] - s->gyro_bias[1];
    float gz = g[2] - s->gyro_bias[2];

    /* Bias EMA chỉ chạy khi ZUPT (gyro đã trừ bias nhỏ hơn ngưỡng) */
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    if (gyro_mag < c->zupt_thr) {
        float k = c->bias_alpha;
        s->gyro_bias[0] = s->gyro_bias[0] + k * gx;
        s->gyro_bias[1] = s->gyro_bias[1] + k * gy;
        s->gyro_bias[2] = s->gyro_bias[2] + k * gz;
    }

    /* dt từ timestamp: hiệu số nguyên uint32 (modulo 2^32) rồi mới đổi sang float */
    float dt = c->nominal_dt;
    if (s->has_ts) {
        uint32_t diff = ts_us - s->last_ts_us;
        float raw_dt = (float)diff * 1e-6f;
        if (c->dt_min <= raw_dt && raw_dt <= c->dt_max) { dt = raw_dt; }
    }
    s->last_ts_us = ts_us;
    s->has_ts = true;

    hap_madgwick_update(&s->mad, c, gx, gy, gz, a[0], a[1], a[2], dt);
    return s->mad.q;
}
