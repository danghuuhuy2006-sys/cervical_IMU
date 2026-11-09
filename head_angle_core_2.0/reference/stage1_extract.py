"""Nạp NGUYÊN VĂN các lớp/hàm lõi từ head_angle_monitor_ble-_final.py (Giai đoạn 1, float64) bằng ast,
không cần cài bleak/matplotlib. Dùng làm đối chứng tuyệt đối cho golden model.
"""
import ast
import math
import queue
import time
import numpy as np

WANT_ASSIGN = {"SAMPLE_RATE_HZ", "CALIB_SECONDS", "LERP_ALPHA", "BETA_MIN", "BETA_MAX", "ACCEL_THR", "ZERO_VEL_THR"}
WANT_DEFS = {"AdaptiveMadgwick", "SensorPipeline", "q_inv", "q_mul", "q_to_euler", "q_to_euler_full",
             "relative_pitch_roll", "calibrate"}


def load(path):
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    ns = {"math": math, "np": np, "queue": queue, "time": time}
    for node in tree.body:
        seg = ast.get_source_segment(src, node)
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) in WANT_ASSIGN for t in node.targets):
            exec(seg, ns)
        elif isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in WANT_DEFS:
            exec(seg, ns)
            if node.name == "calibrate":
                ns["__calibrate_src"] = seg
    return ns


def run_reference(path, rows, calib_n=150):
    """rows: mảng (N, 14) [seq, ts, a1(3), g1(3), a2(3), g2(3)].
    Tự nhận diện bố cục mẫu theo mã nguồn của calibrate():
      - cũ  (head_angle_monitor_ble-_final.py)           : [seq, ts, a1, g1, a2, g2, T1, T2]
      - mới (head_angle_monitor_ble__final_fixed.py)     : [ts, a1, g1, a2, g2, T1, T2, seq]
    relative_pitch_roll trả 2 giá trị (bản trung gian no-update trả 3, có yaw; bản đó không chạy được).
    Trả (mảng [pitch, roll, pitch_lerp, roll_lerp, beta1, beta2, yaw, q1(4), q2(4)], (bias1, bias2)); yaw = NaN nếu bản đó không có."""
    import contextlib, io
    ns = load(path)
    calibrate_src = ns.get("__calibrate_src", "")
    new_layout = "arr[:, 4:7]" in calibrate_src     # bố cục mới: [ts, a1, g1, a2, g2, T1, T2, seq]
    p1 = ns["SensorPipeline"](freq=ns["SAMPLE_RATE_HZ"])
    p2 = ns["SensorPipeline"](freq=ns["SAMPLE_RATE_HZ"])
    dq = queue.Queue()
    for r in rows[:calib_n]:
        seq, ts, imu = r[0], r[1], list(r[2:14])
        dq.put([ts] + imu + [0.0, 0.0, seq] if new_layout else [seq, ts] + imu + [0.0, 0.0])
    with contextlib.redirect_stdout(io.StringIO()):
        qr1, qr2 = ns["calibrate"](p1, p2, dq, ns["CALIB_SECONDS"])
    ui_p = ui_r = 0.0
    out = []
    for r in rows[calib_n:]:
        ts = r[1]
        q1 = p1.process(r[5], r[6], r[7], r[2], r[3], r[4], ts)
        q2 = p2.process(r[11], r[12], r[13], r[8], r[9], r[10], ts)
        q1a = ns["q_mul"](ns["q_inv"](qr1), q1)
        q2a = ns["q_mul"](ns["q_inv"](qr2), q2)
        res = ns["relative_pitch_roll"](q1a, q2a)
        rp, rr = res[0], res[1]
        ry = res[2] if len(res) > 2 else float("nan")
        ui_p = ui_p + ns["LERP_ALPHA"] * (rp - ui_p)
        ui_r = ui_r + ns["LERP_ALPHA"] * (rr - ui_r)
        out.append((rp, rr, ui_p, ui_r, p1.madgwick.beta, p2.madgwick.beta, ry) + tuple(q1) + tuple(q2))
    return np.array(out), (np.array(p1.gyro_bias), np.array(p2.gyro_bias))
