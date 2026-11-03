/* hap_sensor.h - SensorPipeline: trừ bias, bias EMA khi ZUPT, dt từ timestamp, Madgwick. */
#ifndef HAP_SENSOR_H
#define HAP_SENSOR_H
#include "hap_madgwick.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    hap_madgwick_t mad;
    float          gyro_bias[3];
    uint32_t       last_ts_us;
    bool           has_ts;
} hap_sensor_t;

void hap_sensor_init(hap_sensor_t *s, const hap_config_t *cfg);
void hap_sensor_reset(hap_sensor_t *s);          /* reset Madgwick và dấu thời gian, GIỮ bias */
hap_quat_t hap_sensor_process(hap_sensor_t *s, const hap_config_t *cfg,
                              const float g[3], const float a[3], uint32_t ts_us);

#ifdef __cplusplus
}
#endif
#endif
