"""Kiểm tra chính bộ hồi quy: phải ĐẠT với vector đúng và phải FAIL khi vector bị làm sai (dùng bộ stand-in)."""
import json, os, shutil, subprocess, sys, tempfile, hashlib
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "vectors/stage1_standin")
STAGE1 = sys.argv[1]
checks = []


def run(d):
    return subprocess.run([sys.executable, f"{ROOT}/tests/regression_stage1.py", d, "--stage1", STAGE1],
                          capture_output=True, text=True)


def variant(mutate):
    d = tempfile.mkdtemp(); shutil.copytree(SRC, d, dirs_exist_ok=True); mutate(d); return d


def rehash(d, key, fn):
    m = json.load(open(f"{d}/manifest.json")); m[key] = hashlib.sha256(open(f"{d}/{fn}", "rb").read()).hexdigest()
    json.dump(m, open(f"{d}/manifest.json", "w"))


def bump_expected(d):     # sai 0,2 độ ở một mẫu của kết quả Giai đoạn 1 (và cập nhật mã băm để tầng A không bắt trước)
    L = open(f"{d}/expected_stage1.csv").read().splitlines(); r = L[300].split(","); r[2] = repr(float(r[2]) + 0.2); L[300] = ",".join(r)
    open(f"{d}/expected_stage1.csv", "w").write("\n".join(L) + "\n"); rehash(d, "expected_sha256", "expected_stage1.csv")


def tamper_raw(d):        # đổi raw mà không cập nhật mã băm
    L = open(f"{d}/raw.csv").read().splitlines(); r = L[400].split(","); r[5] = "9.9"; L[400] = ",".join(r)
    open(f"{d}/raw.csv", "w").write("\n".join(L) + "\n")


def bump_published(d):    # công bố lệch 0,01 độ (vượt sai số làm tròn 3 chữ số)
    L = open(f"{d}/published_stage1.csv").read().splitlines(); r = L[50].split(","); r[3] = str(float(r[3]) + 0.01); L[50] = ",".join(r)
    open(f"{d}/published_stage1.csv", "w").write("\n".join(L) + "\n"); rehash(d, "published_sha256", "published_stage1.csv")


def check(name, cond):
    checks.append(cond); print(("ok:   " if cond else "FAIL: ") + name)


r = run(SRC); check("vector đúng -> mọi tầng đạt (exit 0)", r.returncode == 0)
r = run(variant(bump_expected)); check("kết quả GĐ1 bị sửa 0,2 độ -> tầng B và C FAIL", r.returncode == 1 and "[B logic f64] FAIL" in r.stdout and "[C float32] FAIL" in r.stdout)
r = run(variant(tamper_raw)); check("raw bị đổi -> tầng A FAIL", r.returncode == 1 and "[A toàn vẹn] FAIL" in r.stdout)
r = run(variant(bump_published)); check("tệp công bố lệch 0,01 độ -> tầng D FAIL", r.returncode == 1 and "[D đã công bố] FAIL" in r.stdout)
print("TẤT CẢ ĐẠT" if all(checks) else "CÓ LỖI"); sys.exit(0 if all(checks) else 1)
