# Component ESP-IDF (ESP32-S3). Cũng biên dịch được bằng gcc trên máy chủ (xem c/host_test/Makefile).
idf_component_register(
    SRCS "src/hap_config.c" "src/hap_quat.c" "src/hap_madgwick.c"
         "src/hap_sensor.c" "src/hap_lerp.c" "src/hap_stream.c"
    INCLUDE_DIRS "include"
)
# Không gộp FMA và không dùng fast-math để kết quả khớp golden model và có thể tái lập.
target_compile_options(${COMPONENT_LIB} PRIVATE -ffp-contract=off -Wall -Wextra -Wdouble-promotion)
