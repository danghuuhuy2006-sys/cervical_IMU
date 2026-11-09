"""Kiểm thử chính API so khớp số: phải ĐẠT khi hai bản khớp và phải FAIL khi sai khác vượt 0,05.
    python3 tests/test_matching.py
"""
import os, shutil, subprocess, sys, tempfile
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from golden.compare import compare, assert_match, match_inputs, host_firmware_runner, NumericalMismatch, main as cli_main

subprocess.check_call(["make", "-C", f"{ROOT}/c/host_test", "-s", "replay"])
REPLAY = f"{ROOT}/c/host_test/replay"
IN = f"{ROOT}/vectors/synthetic_20s_in.csv"
tmp = tempfile.mkdtemp()
G, W = f"{tmp}/golden.csv", f"{tmp}/fw.csv"
from golden.compare import run_golden
run_golden(IN, G)
subprocess.check_call([REPLAY, IN, W])


def edit(src, dst, fn):
    lines = open(src).read().splitlines()
    out, hdr = [], None
    for ln in lines:
        if ln.startswith("#"):
            out.append(ln); continue
        if hdr is None:
            hdr = ln.split(","); out.append(ln); continue
        out.append(fn(hdr, ln.split(",")))
    open(dst, "w").write("\n".join(o if isinstance(o, str) else ",".join(o) for o in out) + "\n")


def add_offset(delta, row_idx=100, col="pitch_raw"):
    def fn(h, r):
        if int(r[0]) == 150 + row_idx:
            r[h.index(col)] = "%.9g" % (float(r[h.index(col)]) + delta)
        return r
    return fn


checks = []
def check(name, cond):
    checks.append(cond); print(("ok:   " if cond else "FAIL: ") + name)

# 1) hai bản thật (Python float32 và C) phải đạt
rep = compare(G, W); check(f"golden vs C đạt (max {max(c.max_abs for c in rep.channels):.1e} <= 0.05)", rep.ok)
check("match_inputs chạy cả hai bản từ cùng đầu vào thô", match_inputs(IN, host_firmware_runner(REPLAY)).ok)

# 2) ngưỡng: lệch 0,04 đạt, 0,06 phải FAIL, đúng bằng 0,05 thì đạt
for d, want in ((0.04, True), (0.06, False), (-0.06, False)):
    p = f"{tmp}/off.csv"; edit(W, p, add_offset(d))
    check(f"lệch {d:+.2f} độ -> {'đạt' if want else 'FAIL'}", compare(G, p).ok == want)
# lệch lớn ở kênh làm mượt và ở mẫu cuối cũng phải bị bắt
p = f"{tmp}/off2.csv"; edit(W, p, add_offset(0.2, row_idx=640, col="roll_smooth"))
check("lệch 0,2 độ ở roll_smooth, mẫu gần cuối -> FAIL", not compare(G, p).ok)
# ngưỡng truyền theo từng kênh
check("ngưỡng riêng từng kênh", compare(G, p, tol={"pitch_raw": .05, "roll_raw": .05, "pitch_smooth": .05, "roll_smooth": .5}).ok)

# 3) lỗi cấu trúc phải FAIL
lines = open(W).read().splitlines(); open(f"{tmp}/short.csv", "w").write("\n".join(lines[:-1]) + "\n")
check("thiếu một dòng -> FAIL", not compare(G, f"{tmp}/short.csv").ok)
def nanify(h, r): r[h.index("pitch_raw")] = "nan" if r[0] == "200" else r[h.index("pitch_raw")]; return r
edit(W, f"{tmp}/nan.csv", nanify); check("có NaN trong firmware -> FAIL", not compare(G, f"{tmp}/nan.csv").ok)
def shift_ts(h, r): r[h.index("ts_us")] = str(int(r[h.index("ts_us")]) + 5) if r[0] == "300" else r[h.index("ts_us")]; return r
edit(W, f"{tmp}/ts.csv", shift_ts); check("ts_us khác (đầu vào khác) -> FAIL", not compare(G, f"{tmp}/ts.csv").ok)
try:
    assert_match(G, f"{tmp}/off2.csv"); check("assert_match phải ném lỗi", False)
except NumericalMismatch:
    check("assert_match ném NumericalMismatch khi fail", True)

# 4) CLI trả mã thoát đúng
check("CLI: mã thoát 0 khi đạt", cli_main([G, W]) == 0)
check("CLI: mã thoát 1 khi fail", cli_main([G, f"{tmp}/off2.csv"]) == 1)

shutil.rmtree(tmp)
print("TẤT CẢ ĐẠT" if all(checks) else "CÓ LỖI"); sys.exit(0 if all(checks) else 1)
