"""Các tình huống biên: gia tốc bằng 0, khoảng trống/ngược thời gian, tràn timestamp uint32, chạy dài.
So khớp PY (float32) với C; REF (float64, Giai đoạn 1) cho thấy mức lệch do float32 và do xử lý tràn timestamp.
    python3 tests/test_edge_cases.py <stage1.py>
"""
import os, subprocess, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from tools.gen_synthetic import make, write
from reference.stage1_extract import run_reference
from golden.replay import main as golden_replay

STAGE1 = sys.argv[1]
WORK = "/tmp/hap_edge"
os.makedirs(WORK, exist_ok=True)
subprocess.check_call(["make", "-C", f"{ROOT}/c/host_test", "-s"])


def run_all(name, cols, with_ref=True):
    csv_in = f"{WORK}/{name}.csv"
    write(csv_in, cols)
    py_out, c_out = f"{WORK}/{name}_py.csv", f"{WORK}/{name}_c.csv"
    golden_replay(csv_in, py_out)
    subprocess.check_call([f"{ROOT}/c/host_test/replay", csv_in, c_out])
    py = np.loadtxt(py_out, delimiter=",", skiprows=2)
    cc = np.loadtxt(c_out, delimiter=",", skiprows=2)
    d_pc = np.abs(py[:, 2:6] - cc[:, 2:6]).max()
    msg = f"{name:28s} PY vs C max = {d_pc:9.2e} độ"
    if with_ref:
        rows = np.loadtxt(csv_in, delimiter=",", skiprows=1)
        rows[:, 2:] = rows[:, 2:].astype(np.float32).astype(np.float64)
        ref, _ = run_reference(STAGE1, rows)
        d_rp = np.abs(ref[:, :4] - py[:, 2:6]).max()
        msg += f" | REF vs PY max = {d_rp:9.2e} độ"
    ok = np.isfinite(py[:, 2:6]).all() and np.isfinite(cc[:, 2:6]).all()
    print(msg, "| hữu hạn:", ok)
    return d_pc, ok


base = make(20.0)

# 1) gia tốc cảm biến đầu bằng 0 trong 6 mẫu (nhánh n < 1e-6, chỉ lan truyền gyro)
c1 = base.copy(); c1[300:306, 8:11] = 0.0
r1 = run_all("accel_zero", c1)

# 2) khoảng trống 0,5 s và timestamp đi lùi 1 s (dt rơi ra ngoài [5 ms, 100 ms] -> dt mặc định)
c2 = base.copy(); c2[400:, 1] += 500_000; c2[500:, 1] -= 1_500_000
r2 = run_all("ts_gap_and_backwards", c2)

# 3) tràn timestamp uint32 giữa mẫu 599 và 600
c3 = base.copy()
shift = (1 << 32) - c3[600, 1] - 7_000          # mẫu 600 nằm sau điểm tràn
c3[:, 1] = np.mod(c3[:, 1] + shift, float(1 << 32))
r3 = run_all("ts_wraparound", c3)

# 4) chạy dài 10 phút: float32 có tích lũy sai lệch so với float64 không?
r4 = run_all("long_600s", make(600.0))

assert all(ok for _, ok in (r1, r2, r3, r4))
assert max(d for d, _ in (r1, r2, r3, r4)) < 1e-3, "PY và C lệch quá 1e-3 độ"
print("OK")
