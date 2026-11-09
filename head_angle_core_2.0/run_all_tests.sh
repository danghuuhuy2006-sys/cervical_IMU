#!/bin/sh
# Chạy toàn bộ kiểm thử. Dùng: ./run_all_tests.sh <head_angle_monitor_ble__final_fixed.py> [<bản cũ ..._-final.py>]
set -e
cd "$(dirname "$0")"
S1="$1"; OLD="$2"
[ -n "$S1" ] || { echo "thiếu đường dẫn script Giai đoạn 1 (bản cuối)"; exit 2; }
make -C c/host_test -s test
python3 tests/test_equivalence.py "$S1" 60 $OLD
python3 tests/test_edge_cases.py "$S1"
python3 tests/test_matching.py
python3 tests/test_regression_harness.py "$S1"
python3 tests/regression_stage1.py vectors/stage1_standin --stage1 "$S1"
python3 tests/test_datagen.py
echo "== TẤT CẢ BỘ KIỂM THỬ ĐẠT =="
