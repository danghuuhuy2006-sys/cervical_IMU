"""Test hồi quy Giai đoạn 1: đưa dữ liệu thô gốc vào mô hình đã đóng gói và kiểm đầu ra có khớp kết quả Giai đoạn 1.

    python3 tests/regression_stage1.py vectors/stage1/session_001 [--no-c] [--tol-logic 1e-9] [--tol-fw 0.05]

Thư mục vector gồm raw.csv, expected_stage1.csv, manifest.json (và published_stage1.csv nếu có), tạo bởi
tools/record_stage1_vectors.py. Bốn tầng kiểm tra:
  A. Toàn vẹn   : mã băm của raw / expected / script Giai đoạn 1 khớp manifest.
  B. Logic      : golden chạy ở float64 phải ra CÙNG SỐ với Giai đoạn 1 (mặc định sai khác <= 1e-9 độ).
                  Đây là bằng chứng cấu trúc lại không đổi logic. Float32 không thể "cùng số" vì Giai đoạn 1 là float64.
  C. Độ chính xác: golden float32 (và mã C nếu đã biên dịch) so với Giai đoạn 1 trong ngưỡng --tol-fw (mặc định 0,05 độ).
  D. Đã công bố : nếu có tệp CSV mà chính script Giai đoạn 1 đã lưu, so với nó (làm tròn 3 chữ số thập phân -> 5,1e-4).
Thoát mã 0 nếu mọi tầng đạt, mã 1 nếu có tầng fail.
"""
import argparse, hashlib, json, os, subprocess, sys, tempfile
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from golden.compare import compare

ap = argparse.ArgumentParser()
ap.add_argument("vector_dir")
ap.add_argument("--no-c", action="store_true")
ap.add_argument("--tol-logic", type=float, default=1e-9)
ap.add_argument("--tol-fw", type=float, default=0.05)
ap.add_argument("--stage1", help="đường dẫn script Giai đoạn 1 để kiểm mã băm (tùy chọn)")
a = ap.parse_args()
D = a.vector_dir
man = json.load(open(f"{D}/manifest.json"))
results = []


def report(tier, ok, msg):
    results.append(ok); print(f"[{tier}] {'ĐẠT ' if ok else 'FAIL'} {msg}")


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


# A. Toàn vẹn
ok = sha(f"{D}/raw.csv") == man["raw_sha256"] and sha(f"{D}/expected_stage1.csv") == man["expected_sha256"]
if a.stage1:
    ok = ok and sha(a.stage1) == man["stage1_script_sha256"]
report("A toàn vẹn", ok, "mã băm raw/expected" + ("/script GĐ1" if a.stage1 else "") + " khớp manifest")

tmp = tempfile.mkdtemp()


def run_golden(dtype, out):
    env = dict(os.environ, HAP_DTYPE=dtype)
    subprocess.check_call([sys.executable, "-m", "golden.replay", f"{D}/raw.csv", out], cwd=ROOT, env=env)


# B. Logic (float64)
run_golden("float64", f"{tmp}/g64.csv")
rep = compare(f"{tmp}/g64.csv", f"{D}/expected_stage1.csv", tol=a.tol_logic)
# compare(golden, firmware) đối xứng về sai khác; ở đây "firmware" là kết quả Giai đoạn 1
report("B logic f64", rep.ok, f"golden float64 vs Giai đoạn 1: max |Δ| = {max(c.max_abs for c in rep.channels):.2e} độ (ngưỡng {a.tol_logic:g})")
if not rep.ok:
    print(rep.to_text())

# C. Độ chính xác float32 và mã C
run_golden("float32", f"{tmp}/g32.csv")
rep = compare(f"{tmp}/g32.csv", f"{D}/expected_stage1.csv", tol=a.tol_fw)
report("C float32", rep.ok, f"golden float32 vs Giai đoạn 1: max |Δ| = {max(c.max_abs for c in rep.channels):.2e} độ (ngưỡng {a.tol_fw:g})")
if not a.no_c:
    subprocess.check_call(["make", "-C", f"{ROOT}/c/host_test", "-s", "replay"])
    subprocess.check_call([f"{ROOT}/c/host_test/replay", f"{D}/raw.csv", f"{tmp}/c.csv"])
    rep = compare(f"{tmp}/c.csv", f"{D}/expected_stage1.csv", tol=a.tol_fw)
    report("C mã C", rep.ok, f"mã C vs Giai đoạn 1: max |Δ| = {max(c.max_abs for c in rep.channels):.2e} độ (ngưỡng {a.tol_fw:g})")

# D. Đối chiếu tệp đã công bố (CSV do script Giai đoạn 1 tự lưu)
if man.get("published"):
    import csv
    pub = list(csv.DictReader(open(f"{D}/published_stage1.csv")))
    exp = list(csv.DictReader(open(f"{D}/expected_stage1.csv")))
    n = len(pub)
    problems = []
    if n == 0 or n > len(exp):
        problems.append(f"số dòng công bố {n} không hợp lệ so với {len(exp)} dòng mong đợi")
    else:
        pairs = [("rel_pitch_deg", "pitch_raw", 5.1e-4), ("rel_roll_deg", "roll_raw", 5.1e-4),
                 ("avg_pitch_deg", "avg_pitch", 5.1e-4), ("avg_roll_deg", "avg_roll", 5.1e-4),
                 ("madgwick_beta1", "beta1", 5.1e-5), ("madgwick_beta2", "beta2", 5.1e-5)]
        pairs += [(f"q{s}{c}", f"q{s}{c}", 5.1e-6) for s in "12" for c in "wxyz"]
        worst = {}
        for pc, ec, tol in pairs:
            d = max(abs(float(p[pc]) - float(e[ec])) for p, e in zip(pub, exp))
            worst[pc] = d
            if d > tol:
                problems.append(f"{pc}: max |Δ| = {d:.3e} > {tol:g}")
        dts = max(abs(float(p["timestamp_s"]) - float(e["ts_us"]) * 1e-6) for p, e in zip(pub, exp))
        if dts > 5.1e-5:
            problems.append(f"timestamp_s lệch {dts:.2e} s: hai tệp không cùng phiên")
    report("D đã công bố", not problems, f"{n} dòng CSV Giai đoạn 1 vs đầu ra mong đợi" + ("" if not problems else " | " + "; ".join(problems)))
else:
    print("[D đã công bố] BỎ QUA: manifest không có tệp CSV đã công bố (--published khi ghi vector)")

print("\nTẤT CẢ ĐẠT" if all(results) else "\nCÓ TẦNG FAIL")
sys.exit(0 if all(results) else 1)
