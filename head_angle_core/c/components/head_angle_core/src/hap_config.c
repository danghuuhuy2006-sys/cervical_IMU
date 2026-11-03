#include "hap_types.h"

void hap_config_default(hap_config_t *cfg)
{
    cfg->beta_min          = 0.02f;
    cfg->beta_max          = 0.08f;
    cfg->accel_thr         = 0.08f;
    cfg->zupt_thr          = 0.002f;
    cfg->bias_alpha        = 2e-4f;
    cfg->nominal_dt        = 1.0f / 50.0f;
    cfg->dt_min            = 0.005f;
    cfg->dt_max            = 0.1f;
    cfg->lerp_alpha        = 0.25f;
    cfg->calib_samples     = 150u;
    cfg->warmup_max_rounds = 15u;
    cfg->warmup_tol        = 0.0005f;
    cfg->bias_std_warn     = 0.005f;
}
