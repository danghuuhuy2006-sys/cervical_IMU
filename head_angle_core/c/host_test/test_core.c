/* test_core.c - kiểm thử đơn vị lõi C (chạy trên máy chủ): make test */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "hap_stream.h"

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } else { printf("ok:   %s\n", msg); } } while (0)

int main(void)
{
    hap_config_t cfg; hap_config_default(&cfg);

    /* quaternion */
    hap_quat_t id = hap_q_identity(), q = { 0.5f, 0.5f, 0.5f, 0.5f };
    hap_quat_t r = hap_q_mul(hap_q_inv(q), q);
    CHECK(fabsf(r.w - 1.0f) < 1e-6f && fabsf(r.x) < 1e-6f && fabsf(r.y) < 1e-6f && fabsf(r.z) < 1e-6f, "q^-1 * q = identity");
    float p, ro;
    hap_q_to_euler(id, &p, &ro);
    CHECK(p == 0.0f && ro == 0.0f, "euler(identity) = 0");
    hap_quat_t qc = { 0.70710683f, 0.0f, 0.70710683f, 0.0f };      /* 2*(w*y) = 1.0000001 -> phải kẹp về 90 độ */
    hap_q_to_euler(qc, &p, &ro);
    CHECK(fabsf(p - 90.0f) < 1e-3f && isfinite(p), "asin được kẹp trong [-1,1]");

    /* LERP */
    hap_lerp_t l; hap_lerp_init(&l, 0.25f, 0.0f);
    CHECK(hap_lerp_step(&l, 4.0f) == 1.0f && hap_lerp_step(&l, 4.0f) == 1.75f, "LERP nhân quả: 0 -> 1 -> 1.75");
    CHECK(fabsf(hap_lerp_interp(1.5f, 1.0f, 2.0f, 10.0f, 20.0f) - 15.0f) < 1e-5f, "LERP nội suy tại điểm giữa");

    /* tràn timestamp uint32: kết quả phải giống hệt khi không tràn */
    hap_sensor_t sa, sb; hap_sensor_init(&sa, &cfg); hap_sensor_init(&sb, &cfg);
    float g[3] = { 0.3f, -0.2f, 0.1f }, a[3] = { 0.0f, 0.1f, 0.99f };
    uint32_t ta = 4294960000u, tb = 1000000u;
    hap_quat_t qa = {0}, qb = {0};
    for (int i = 0; i < 20; i++) {
        qa = hap_sensor_process(&sa, &cfg, g, a, ta);
        qb = hap_sensor_process(&sb, &cfg, g, a, tb);
        ta += 20000u; tb += 20000u;                         /* ta đi qua 2^32 giữa chừng */
    }
    CHECK(memcmp(&qa, &qb, sizeof qa) == 0, "tràn timestamp uint32 không đổi kết quả");

    /* máy trạng thái hiệu chuẩn */
    static hap_stream_t s; hap_stream_init(&s, &cfg);
    hap_sample_t x; memset(&x, 0, sizeof x);
    x.a1[2] = 1.0f; x.a2[2] = 1.0f;
    hap_output_t o;
    CHECK(!hap_stream_step(&s, &x, &o), "step trước khi hiệu chuẩn bị từ chối");
    hap_state_t st = HAP_STATE_COLLECTING;
    for (uint32_t i = 0; i < cfg.calib_samples; i++) { x.ts_us = i * 20000u; st = hap_stream_push_calib(&s, &x); }
    CHECK(st == HAP_STATE_WARMUP, "đủ mẫu thì chuyển sang WARMUP");
    int rounds = 0; while (!hap_stream_warmup_round(&s)) { rounds++; }
    CHECK(s.state == HAP_STATE_RUNNING && hap_stream_report(&s)->rounds <= cfg.warmup_max_rounds, "warm-up kết thúc trong số vòng tối đa");
    x.ts_us = cfg.calib_samples * 20000u;
    CHECK(hap_stream_step(&s, &x, &o) && fabsf(o.pitch_raw) < 0.01f && fabsf(o.roll_raw) < 0.01f, "dữ liệu tĩnh sau hiệu chuẩn cho góc ~ 0");
    CHECK(o.beta1 >= cfg.beta_min - 1e-6f && o.beta1 <= cfg.beta_max + 1e-6f, "beta nằm trong [beta_min, beta_max]");
    printf("sizeof(hap_stream_t) = %u byte (tĩnh)\n", (unsigned)sizeof(hap_stream_t));

    printf(fails ? "\n%d lỗi\n" : "\ntất cả đạt\n", fails);
    return fails ? 1 : 0;
}
