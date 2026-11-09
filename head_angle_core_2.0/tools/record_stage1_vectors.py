"""Chốt "kết quả Giai đoạn 1" từ một bản ghi thô: chạy CHÍNH code Giai đoạn 1 (float64, nguyên văn) trên dữ liệu đó,
lưu đầu ra mong đợi cùng mã băm để test hồi quy dùng lại.

    python3 tools/record_stage1_vectors.py raw_session_001.csv --stage1 head_angle_monitor_ble__final_fixed.py \
            --out vectors/stage1/session_001 [--published session_001_stage1_output.csv]

Nếu có --published (tệp CSV mà chính script Giai đoạn 1 đã lưu cùng phiên đó), nó được sao vào thư mục để test hồi quy
đối chiếu thêm với kết quả "đã công bố".
"""
import argparse, hashlib, json, os, shutil, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from reference.stage1_extract import run_reference

EXPECTED_HEADER = ("idx,ts_us,pitch_raw,roll_raw,pitch_smooth,roll_smooth,beta1,beta2,"
                   "q1w,q1x,q1y,q1z,q2w,q2x,q2y,q2z,avg_pitch,avg_roll")


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def main(raw, stage1, out, published=None, calib_n=150):
    os.makedirs(out, exist_ok=True)
    rows = np.loadtxt(raw, delimiter=",", skiprows=1, dtype=np.float64)
    if len(rows) <= calib_n:
        raise SystemExit(f"bản ghi chỉ có {len(rows)} mẫu, cần nhiều hơn {calib_n} mẫu hiệu chuẩn")
    ref, _ = run_reference(stage1, rows, calib_n=calib_n)
    raw_out = os.path.join(out, "raw.csv")
    shutil.copyfile(raw, raw_out)
    exp = os.path.join(out, "expected_stage1.csv")
    with open(exp, "w") as f:
        f.write(EXPECTED_HEADER + "\n")
        sp = sr = 0.0
        for k, (r, row) in enumerate(zip(ref, rows[calib_n:]), start=1):
            sp += r[0]; sr += r[1]                      # trung bình cộng dồn như avg_pitch_deg của Giai đoạn 1
            f.write(("%d,%d," + ",".join(["%.17g"] * 16) + "\n") % (
                calib_n + k - 1, row[1], r[0], r[1], r[2], r[3], r[4], r[5], *r[7:15], sp / k, sr / k))
    man = {"raw": "raw.csv", "raw_sha256": sha(raw_out), "expected": "expected_stage1.csv", "expected_sha256": sha(exp),
           "stage1_script": os.path.basename(stage1), "stage1_script_sha256": sha(stage1), "calib_samples": calib_n,
           "n_samples": int(len(rows)), "published": None}
    if published:
        pub = os.path.join(out, "published_stage1.csv")
        shutil.copyfile(published, pub)
        man.update(published="published_stage1.csv", published_sha256=sha(pub))
    json.dump(man, open(os.path.join(out, "manifest.json"), "w"), indent=2)
    print(f"đã ghi {out}: {len(rows) - calib_n} mẫu mong đợi")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("raw"); ap.add_argument("--stage1", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--published"); ap.add_argument("--calib-n", type=int, default=150)
    a = ap.parse_args()
    main(a.raw, a.stage1, a.out, a.published, a.calib_n)
