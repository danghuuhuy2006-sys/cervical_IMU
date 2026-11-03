/* replay.c - phát lại tệp CSV qua mã C (hap_stream) trên máy chủ, ghi kết quả cùng định dạng golden.replay.
 *   ./replay in.csv out.csv
 * Dùng strtof để đọc đúng giá trị float32 mà gen_synthetic.py đã ghi (9 chữ số có nghĩa). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hap_stream.h"

static int parse_line(const char *line, hap_sample_t *x)
{
    char *end;
    const char *p = line;
    (void)strtod(p, &end);                 /* seq */
    p = end + 1;
    x->ts_us = (uint32_t)strtoull(p, &end, 10);
    p = end + 1;
    float *dst[12] = { &x->a1[0], &x->a1[1], &x->a1[2], &x->g1[0], &x->g1[1], &x->g1[2],
                       &x->a2[0], &x->a2[1], &x->a2[2], &x->g2[0], &x->g2[1], &x->g2[2] };
    for (int i = 0; i < 12; i++) {
        *dst[i] = strtof(p, &end);
        if (end == p) { return -1; }
        p = (*end == ',') ? end + 1 : end;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s in.csv out.csv\n", argv[0]); return 2; }
    FILE *fi = fopen(argv[1], "r");
    FILE *fo = fopen(argv[2], "w");
    if (!fi || !fo) { perror("open"); return 2; }

    static hap_stream_t s;                       /* ~8 KB, đặt ở vùng tĩnh */
    hap_config_t cfg;
    hap_config_default(&cfg);
    hap_stream_init(&s, &cfg);

    char line[1024];
    if (!fgets(line, sizeof line, fi)) { return 2; }      /* bỏ dòng tiêu đề */
    long idx = 0;
    int header_written = 0;
    while (fgets(line, sizeof line, fi)) {
        hap_sample_t x;
        if (parse_line(line, &x) != 0) { fprintf(stderr, "parse error at row %ld\n", idx); return 2; }
        if (s.state == HAP_STATE_COLLECTING) {
            (void)hap_stream_push_calib(&s, &x);
            if (s.state == HAP_STATE_WARMUP) { while (!hap_stream_warmup_round(&s)) { } }
        } else {
            if (!header_written) {
                const hap_calib_report_t *r = hap_stream_report(&s);
                fprintf(fo, "# calib rounds=%d converged=%d vib=%d bias1=%.9g %.9g %.9g bias2=%.9g %.9g %.9g\n",
                        r->rounds, r->converged, r->vibration_warning,
                        r->bias1[0], r->bias1[1], r->bias1[2], r->bias2[0], r->bias2[1], r->bias2[2]);
                fprintf(fo, "idx,ts_us,pitch_raw,roll_raw,pitch_smooth,roll_smooth,beta1,beta2,zupt1,zupt2,yaw_raw\n");
                header_written = 1;
            }
            hap_output_t o;
            (void)hap_stream_step(&s, &x, &o);
            fprintf(fo, "%ld,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.9g\n", idx, (unsigned)x.ts_us,
                    (double)o.pitch_raw, (double)o.roll_raw, (double)o.pitch_smooth, (double)o.roll_smooth,
                    (double)o.beta1, (double)o.beta2, o.zupt1, o.zupt2, (double)o.yaw_raw);
        }
        idx++;
    }
    fclose(fi); fclose(fo);
    return 0;
}
