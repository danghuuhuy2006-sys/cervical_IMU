#include "hap_lerp.h"

void hap_lerp_init(hap_lerp_t *l, float alpha, float y0)
{
    l->alpha = alpha;
    l->y = y0;
}

float hap_lerp_step(hap_lerp_t *l, float x)
{
    l->y = l->y + l->alpha * (x - l->y);
    return l->y;
}

float hap_lerp_interp(float t_star, float t_i, float t_ip1, float v_i, float v_ip1)
{
    return v_i + (t_star - t_i) / (t_ip1 - t_i) * (v_ip1 - v_i);
}
