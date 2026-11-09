/* hap_lerp.h - module LERP, tách riêng khỏi chuỗi dung hợp.
 *  - hap_lerp_step: bộ làm mượt NHÂN QUẢ y += alpha*(x - y), chạy từng mẫu (đúng code Giai đoạn 1).
 *  - hap_lerp_interp: công thức 7 (nội suy tuyến tính giữa hai mốc), hàm thuần để đối chiếu chéo.
 * Tái lấy mẫu offline không nhân quả KHÔNG nằm trong firmware (xem golden/resample.py). */
#ifndef HAP_LERP_H
#define HAP_LERP_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float y; float alpha; } hap_lerp_t;

void  hap_lerp_init(hap_lerp_t *l, float alpha, float y0);   /* Giai đoạn 1 dùng y0 = 0 */
float hap_lerp_step(hap_lerp_t *l, float x);
float hap_lerp_interp(float t_star, float t_i, float t_ip1, float v_i, float v_ip1);

#ifdef __cplusplus
}
#endif
#endif
