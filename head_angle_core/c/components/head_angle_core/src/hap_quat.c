#include "hap_quat.h"
#include <math.h>

hap_quat_t hap_q_identity(void)
{
    hap_quat_t q = { 1.0f, 0.0f, 0.0f, 0.0f };
    return q;
}

hap_quat_t hap_q_inv(hap_quat_t q)
{
    hap_quat_t r = { q.w, -q.x, -q.y, -q.z };
    return r;
}

hap_quat_t hap_q_mul(hap_quat_t q, hap_quat_t r)
{
    hap_quat_t o;
    o.w = q.w*r.w - q.x*r.x - q.y*r.y - q.z*r.z;
    o.x = q.w*r.x + q.x*r.w + q.y*r.z - q.z*r.y;
    o.y = q.w*r.y - q.x*r.z + q.y*r.w + q.z*r.x;
    o.z = q.w*r.z + q.x*r.y - q.y*r.x + q.z*r.w;
    return o;
}

void hap_q_to_euler(hap_quat_t q, float *pitch_deg, float *roll_deg)
{
    float t = 2.0f * (q.w*q.y - q.z*q.x);
    if (t > 1.0f)  { t = 1.0f; }
    if (t < -1.0f) { t = -1.0f; }
    *pitch_deg = asinf(t) * HAP_RAD2DEG;
    *roll_deg  = atan2f(2.0f * (q.w*q.x + q.y*q.z),
                        1.0f - 2.0f * (q.x*q.x + q.y*q.y)) * HAP_RAD2DEG;
}

void hap_q_to_euler_full(hap_quat_t q, float *pitch_deg, float *roll_deg, float *yaw_deg)
{
    hap_q_to_euler(q, pitch_deg, roll_deg);
    *yaw_deg = atan2f(2.0f * (q.w*q.z + q.x*q.y), 1.0f - 2.0f * (q.y*q.y + q.z*q.z)) * HAP_RAD2DEG;
}

void hap_relative_euler(hap_quat_t q_base, hap_quat_t q_head, float *pitch_deg, float *roll_deg, float *yaw_deg)
{
    hap_quat_t q_rel = hap_q_mul(hap_q_inv(q_base), q_head);
    if (q_rel.w < 0.0f) {          /* dạng chuẩn: phần thực dương */
        q_rel.w = -q_rel.w; q_rel.x = -q_rel.x; q_rel.y = -q_rel.y; q_rel.z = -q_rel.z;
    }
    hap_q_to_euler_full(q_rel, pitch_deg, roll_deg, yaw_deg);
}

void hap_relative_pitch_roll(hap_quat_t q_base, hap_quat_t q_head, float *pitch_deg, float *roll_deg)
{
    float yaw;
    hap_relative_euler(q_base, q_head, pitch_deg, roll_deg, &yaw);
}

float hap_q_dist(hap_quat_t a, hap_quat_t b)
{
    float d0 = a.w - b.w, d1 = a.x - b.x, d2 = a.y - b.y, d3 = a.z - b.z;
    return sqrtf(d0*d0 + d1*d1 + d2*d2 + d3*d3);
}
