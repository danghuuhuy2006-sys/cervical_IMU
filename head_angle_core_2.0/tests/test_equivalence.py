"""So khớp ba bản cài đặt trên cùng luồng dữ liệu:
   REF = code Giai đoạn 1 nguyên văn (float64)
   PY  = golden model float32 (golden/)
   C   = mã C float (c/components/head_angle_core), biên dịch trên máy chủ

    python3 tests/test_equivalence.py <head_angle_monitor_ble-_final.py> [duration_s]
"""
import os
import subprocess
import sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from tools.gen_synthetic import make, write                      # noqa: E402
from reference.stage1_extract import run_reference               # noqa: E402
from golden.replay import main as golden_replay                  # noqa: E402

STAGE1 = sys.argv[1]
STAGE1_OLD = sys.argv[3] if len(sys.argv) > 3 else None
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
WORK = "/tmp/hap_eq"
os.makedirs(WORK, exist_ok=True)

cols = make(DUR)
csv_in = f"{WORK}/in.csv"
write(csv_in, cols)

# 1) REF float64, nhưng nhận đúng giá trị float32 mà hai bản kia đọc từ tệp
rows = np.loadtxt(csv_in, delimiter=",", skiprows=1)
rows[:, 2:] = rows[:, 2:].astype(np.float32).astype(np.float64)
ref, _ = run_reference(STAGE1, rows)
if STAGE1_OLD:
    old, _ = run_reference(STAGE1_OLD, rows)
    print(f"Giai đoạn 1 cũ và bản cuối (float64): max |pitch,roll,LERP| lệch = {np.abs(old[:, :4] - ref[:, :4]).max():.3e} độ  (0 nghĩa là lõi không đổi)")

# 2) PY float32
py_out = f"{WORK}/py.csv"
golden_replay(csv_in, py_out)

# 3) C float
subprocess.check_call(["make", "-C", f"{ROOT}/c/host_test", "-s"])
c_out = f"{WORK}/c.csv"
subprocess.check_call([f"{ROOT}/c/host_test/replay", csv_in, c_out])


def load(p):
    return np.loadtxt(p, delimiter=",", skiprows=2)


py, cc = load(py_out), load(c_out)
assert len(ref) == len(py) == len(cc), (len(ref), len(py), len(cc))
names = ["pitch_raw", "roll_raw", "pitch_smooth", "roll_smooth", "yaw_raw"]
col = {"pitch_raw": 2, "roll_raw": 3, "pitch_smooth": 4, "roll_smooth": 5, "yaw_raw": 10}


def stat(a, b):
    d = np.abs(a - b)
    return d.max(), np.sqrt(np.mean(d * d))


print(f"Số mẫu sau hiệu chuẩn: {len(ref)}   (nguồn mô phỏng, {DUR:.0f} s)")
print(f"{'kênh':14s} | {'REF vs PY':>22s} | {'PY vs C':>22s} | {'REF vs C':>22s}   (max / RMS, độ)")
worst = {"ref_py": 0, "py_c": 0, "ref_c": 0}
for i, nme in enumerate(names):
    a = ref[:, 6] if nme == "yaw_raw" else ref[:, i]
    b, c = py[:, col[nme]], cc[:, col[nme]]
    if nme == "yaw_raw" and np.isnan(a).all():          # bản Giai đoạn 1 này không xuất yaw
        s2 = stat(b, c)
        worst["py_c"] = max(worst["py_c"], s2[0])
        print(f"{nme:14s} | {'(không có ở GĐ1)':>22s} | {s2[0]:10.2e} / {s2[1]:9.2e} | {'':>22s}")
        continue
    s1, s2, s3 = stat(a, b), stat(b, c), stat(a, c)
    worst["ref_py"] = max(worst["ref_py"], s1[0]); worst["py_c"] = max(worst["py_c"], s2[0]); worst["ref_c"] = max(worst["ref_c"], s3[0])
    print(f"{nme:14s} | {s1[0]:10.2e} / {s1[1]:9.2e} | {s2[0]:10.2e} / {s2[1]:9.2e} | {s3[0]:10.2e} / {s3[1]:9.2e}")
bd = max(np.abs(ref[:, 4] - py[:, 6]).max(), np.abs(ref[:, 5] - py[:, 7]).max())  # beta1, beta2
print(f"beta: max |REF - PY| = {bd:.2e}")
print(f"Khoảng giá trị pitch_raw: [{ref[:,0].min():.1f}, {ref[:,0].max():.1f}] độ; roll_raw: [{ref[:,1].min():.1f}, {ref[:,1].max():.1f}] độ")
print(f"ZUPT kích hoạt: cảm biến 1 {int(py[:,8].sum())}/{len(py)} mẫu, cảm biến 2 {int(py[:,9].sum())}/{len(py)} mẫu")
print("Báo cáo hiệu chuẩn (PY):", open(py_out).readline().strip())
print("Báo cáo hiệu chuẩn (C) :", open(c_out).readline().strip())
print("worst:", {k: f"{v:.2e}" for k, v in worst.items()})
