#include "hap_stream.h"
#include <math.h>

static void mean_std3(const hap_sample_t *buf, uint16_t n, bool head, float mean[3], float sd[3])
{
    const float nf = (float)n;
    for (int k = 0; k < 3; k++) {
        float sum = 0.0f;
        for (uint16_t i = 0; i < n; i++) {
            sum = sum + (head ? buf[i].g2[k] : buf[i].g1[k]);
        }
        float m = sum / nf;
        float acc = 0.0f;
        for (uint16_t i = 0; i < n; i++) {
            float d = (head ? buf[i].g2[k] : buf[i].g1[k]) - m;
            acc = acc + d * d;
        }
        mean[k] = m;
        sd[k] = sqrtf(acc / nf);
    }
}

void hap_stream_init(hap_stream_t *s, const hap_config_t *cfg)
{
    s->cfg = *cfg;
    if (s->cfg.calib_samples > HAP_CALIB_MAX_SAMPLES) { s->cfg.calib_samples = HAP_CALIB_MAX_SAMPLES; }
    hap_sensor_init(&s->s1, &s->cfg);
    hap_sensor_init(&s->s2, &s->cfg);
    s->qr1 = hap_q_identity();
    s->qr2 = hap_q_identity();
    hap_lerp_init(&s->lp, s->cfg.lerp_alpha, 0.0f);
    hap_lerp_init(&s->lr, s->cfg.lerp_alpha, 0.0f);
    s->state = HAP_STATE_COLLECTING;
    s->calib_n = 0u;
    s->report = (hap_calib_report_t){ {0}, {0}, {0}, {0}, 0.0f, 0.0f, 0u, false, false };
}

hap_state_t hap_stream_push_calib(hap_stream_t *s, const hap_sample_t *x)
{
    if (s->state != HAP_STATE_COLLECTING) { return s->state; }
    s->calib_buf[s->calib_n++] = *x;
    if (s->calib_n < s->cfg.calib_samples) { return s->state; }

    mean_std3(s->calib_buf, s->calib_n, false, s->report.bias1, s->report.std1);
    mean_std3(s->calib_buf, s->calib_n, true,  s->report.bias2, s->report.std2);
    bool warn = false;
    for (int k = 0; k < 3; k++) {
        s->s1.gyro_bias[k] = s->report.bias1[k];
        s->s2.gyro_bias[k] = s->report.bias2[k];
        if (s->report.std1[k] > s->cfg.bias_std_warn || s->report.std2[k] > s->cfg.bias_std_warn) { warn = true; }
    }
    s->report.vibration_warning = warn;
    s->report.rounds = 0u;
    s->report.converged = false;
    hap_sensor_reset(&s->s1);
    hap_sensor_reset(&s->s2);
    s->state = HAP_STATE_WARMUP;
    return s->state;
}

bool hap_stream_warmup_round(hap_stream_t *s)
{
    if (s->state != HAP_STATE_WARMUP) { return s->state == HAP_STATE_RUNNING; }
    hap_quat_t p1 = s->s1.mad.q, p2 = s->s2.mad.q;
    for (uint16_t i = 0; i < s->calib_n; i++) {
        const hap_sample_t *v = &s->calib_buf[i];
        (void)hap_sensor_process(&s->s1, &s->cfg, v->g1, v->a1, v->ts_us);
        (void)hap_sensor_process(&s->s2, &s->cfg, v->g2, v->a2, v->ts_us);
    }
    float d1 = hap_q_dist(s->s1.mad.q, p1);
    float d2 = hap_q_dist(s->s2.mad.q, p2);
    s->report.rounds = (uint8_t)(s->report.rounds + 1u);
    s->report.dq1 = d1;
    s->report.dq2 = d2;
    s->report.converged = (d1 < s->cfg.warmup_tol) && (d2 < s->cfg.warmup_tol);
    if (s->report.converged || s->report.rounds >= s->cfg.warmup_max_rounds) {
        s->qr1 = s->s1.mad.q;
        s->qr2 = s->s2.mad.q;
        hap_lerp_init(&s->lp, s->cfg.lerp_alpha, 0.0f);
        hap_lerp_init(&s->lr, s->cfg.lerp_alpha, 0.0f);
        s->state = HAP_STATE_RUNNING;
        return true;
    }
    return false;
}

bool hap_stream_step(hap_stream_t *s, const hap_sample_t *x, hap_output_t *out)
{
    if (s->state != HAP_STATE_RUNNING) { return false; }
    hap_quat_t q1 = hap_sensor_process(&s->s1, &s->cfg, x->g1, x->a1, x->ts_us);
    hap_quat_t q2 = hap_sensor_process(&s->s2, &s->cfg, x->g2, x->a2, x->ts_us);
    hap_quat_t q1_adj = hap_q_mul(hap_q_inv(s->qr1), q1);
    hap_quat_t q2_adj = hap_q_mul(hap_q_inv(s->qr2), q2);
    float rp, rr, ry;
    hap_relative_euler(q1_adj, q2_adj, &rp, &rr, &ry);
    out->pitch_raw = rp;
    out->roll_raw = rr;
    out->pitch_smooth = hap_lerp_step(&s->lp, rp);
    out->roll_smooth = hap_lerp_step(&s->lr, rr);
    out->q1 = q1;
    out->q2 = q2;
    out->beta1 = s->s1.mad.beta;
    out->beta2 = s->s2.mad.beta;
    out->zupt1 = s->s1.mad.zupt;
    out->zupt2 = s->s2.mad.zupt;
    out->yaw_raw = ry;
    return true;
}

void hap_stream_tare(hap_stream_t *s)
{
    s->qr1 = s->s1.mad.q;
    s->qr2 = s->s2.mad.q;
}

const hap_calib_report_t *hap_stream_report(const hap_stream_t *s)
{
    return &s->report;
}
