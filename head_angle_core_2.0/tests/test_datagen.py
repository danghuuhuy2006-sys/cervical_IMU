"""Kiểm thử bộ sinh dữ liệu: dải sinh lý, tốc độ, quy đổi d/r, STA tăng theo biên độ/tốc độ, nhóm BMI, nhiễu cảm biến,
tính nhất quán với golden model, nhãn, tái lập, chia theo người, ngân hàng quỹ đạo, tương đương nhãn Python và C.
    python3 tests/test_datagen.py
"""
import os, subprocess, sys, tempfile
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from datagen import Params, sample_subject, simulate_sequence, make_dataset, loso_splits
from datagen.trajectory import gen_axis_track, sta_angle_deg, TrajectoryBank
from datagen.quat import from_rotvec, to_rotvec

checks = []
def check(name, cond, info=""):
    checks.append(bool(cond)); print(("ok:   " if cond else "FAIL: ") + name + (f"   [{info}]" if info else ""))

P = Params()
rng0 = np.random.default_rng(0)
sub = sample_subject(rng0, "S000", "mid", P)

# 1) dải sinh lý: không vượt, và bao phủ ---------------------------------------------------------------------
fs, n, k0 = P.fs_sim, 120 * P.fs_sim, 4 * P.fs_sim
lims = {"pitch": P.physio.pitch_deg, "roll": tuple(np.array(P.physio.roll_deg) * P.physio.roll_scale)}
tr = {k: [] for k in lims}
peak = 0.0
for s in range(60):
    r = np.random.default_rng(1000 + s)
    for k in lims:
        x = gen_axis_track(r, n, fs, lims[k], P.motion, P.physio, k0)
        tr[k].append(x); peak = max(peak, np.abs(np.gradient(x, 1 / fs)).max())
for k, lim in lims.items():
    x = np.concatenate(tr[k])
    check(f"{k}: không vượt dải sinh lý {lim}", x.min() >= lim[0] - 1e-9 and x.max() <= lim[1] + 1e-9, f"min {x.min():.1f}, max {x.max():.1f}")
    check(f"{k}: bao phủ >= 90% mỗi phía của dải", x.min() <= 0.9 * lim[0] and x.max() >= 0.9 * lim[1])
check(f"tốc độ góc tương đối <= {P.physio.v_peak_max_dps:.0f} °/s", peak <= P.physio.v_peak_max_dps * 1.01, f"đỉnh {peak:.1f} °/s")
check("không có khoảng 4 s đầu chuyển động (đứng yên cho hiệu chuẩn)", all(np.abs(t[:k0]).max() == 0 for t in tr["pitch"]))

# 2) quy đổi d / r ---------------------------------------------------------------------------------------------
check("d/r: 13,5 mm với r = 100 mm -> 7,73 độ", abs(sta_angle_deg(13.5, 0.10) - 7.735) < 0.01, f"{sta_angle_deg(13.5, 0.10):.3f}")
check("d/r: 21 mm với r = 100 mm -> 12,03 độ", abs(sta_angle_deg(21.0, 0.10) - 12.03) < 0.01)
phi = np.array([0.01, -0.02, 0.03]); check("vector quay <-> quaternion khứ hồi", np.allclose(to_rotvec(from_rotvec(phi)), phi, atol=1e-12))

# 3) nhóm nhân trắc: tỉ lệ STA ~ 0,7 : 1 : 1,27 ----------------------------------------------------------------------
means = {}
for g in ("low", "mid", "high"):
    r = np.random.default_rng(5)
    means[g] = np.mean([sample_subject(r, "x", g, P).sta_scale for _ in range(400)])
rat = (means["low"] / means["mid"], means["high"] / means["mid"])
check("tỉ lệ STA theo BMI ~ 0,70 và 1,27 so với nhóm giữa (sai số <= 8%)", abs(rat[0] - 0.70) < 0.056 and abs(rat[1] - 1.275) < 0.10, f"{rat[0]:.2f}, {rat[1]:.2f}")

# 4) STA tăng theo biên độ và tốc độ -----------------------------------------------------------------------------------
s = simulate_sequence(sub, P, seed=11, seconds=120)
sta = np.linalg.norm(s["sta_head_deg"], axis=1)
speed = np.abs(np.gradient(s["rel_pitch_true"], 1 / P.fs_out))
amp = np.abs(s["rel_pitch_true"])
still = (speed < 1.0) & (amp < 1.0)
move = (speed > np.percentile(speed, 85)) & (amp > np.percentile(amp, 60))
check("STA lớn hơn rõ khi chuyển động nhanh và biên độ lớn so với khi nghỉ", sta[move].mean() > 3.0 * sta[still].mean(),
      f"{sta[move].mean():.2f}° so với {sta[still].mean():.2f}°")

ana_all = np.concatenate([s["theta_anat"]])
check("theta_anat (góc xương giải phẫu) nằm trong dải sinh lý", ana_all[:, 0].min() >= P.physio.pitch_deg[0] - 0.05 and ana_all[:, 0].max() <= P.physio.pitch_deg[1] + 0.05
      and np.abs(ana_all[:, 1]).max() <= P.physio.roll_deg[1] + 0.05, f"pitch {ana_all[:,0].min():.1f}..{ana_all[:,0].max():.1f}, roll ±{np.abs(ana_all[:,1]).max():.1f}")

# 5) nhiễu cảm biến (tách STA ra) -----------------------------------------------------------------------------------------
sn = simulate_sequence(sub, P, seed=3, seconds=30, sta_on=False, mount_on=False)
g = sn["g1"][:150].astype(np.float64)
check("độ lệch chuẩn gyro lúc tĩnh ~ gyro_white (±20%)", abs(g.std(0).mean() / P.noise.gyro_white_rads - 1) < 0.2, f"{g.std(0).mean():.2e} so với {P.noise.gyro_white_rads:.1e}")
lsb = 1 / (P.noise.gyro_lsb_per_dps * 180 / np.pi)
check("gyro nằm đúng lưới lượng tử hóa 1/131 °/s", np.allclose(np.round(sn["g2"] / lsb) * lsb, sn["g2"], atol=1e-6))
check("gia tốc nằm trong ±2 g và gyro trong ±250 °/s", np.abs(sn["a2"]).max() <= 2.0 and np.degrees(np.abs(sn["g2"])).max() <= 250.0)

# 6) nhất quán với golden: không STA, nhiễu, lệch -> theta_base bám theta_ref -------------------------------------------------
c0 = simulate_sequence(sub, P, seed=5, seconds=60, sta_on=False, noise_on=False, mount_on=False)
d0 = c0["delta"][c0["valid"]]
rms0 = np.sqrt((d0 ** 2).mean(0))
check("lý tưởng (không STA/nhiễu/lệch): RMS delta <= 0,5 độ, max <= 3 độ", rms0.max() <= 0.5 and np.abs(d0).max() <= 3.0, f"RMS {rms0.round(2)}, max {np.abs(d0).max():.2f}")

# 7) STA làm tăng delta, và tăng theo hệ số ---------------------------------------------------------------------------------
def rms(pp, **kw):
    q = simulate_sequence(sub, pp, seed=5, seconds=60, **kw); d = q["delta"][q["valid"]]; return np.sqrt((d ** 2).mean())
r_off, r_on = rms(P, sta_on=False), rms(P)
p_half = Params(); p_half.sta.tilt_coupling = 0.5
r_half = rms(p_half)
check("bật STA làm RMS delta tăng >= 3 lần (nhãn tách riêng STA, lệch lắp đặt không nằm trong nhãn)", r_on > 3.0 * r_off, f"{r_off:.2f}° -> {r_on:.2f}°")
cm = simulate_sequence(sub, P, seed=5, seconds=60, sta_on=False, noise_on=False, mount_on=True)
dm = cm["delta"][cm["valid"]]
check("lệch lắp đặt 3° không làm delta lớn (theta_ref là cảm biến gắn cứng): RMS <= 0,5 độ", np.sqrt((dm ** 2).mean(0)).max() <= 0.5, f"RMS {np.sqrt((dm ** 2).mean(0)).round(2)}")
ana = np.sqrt(((cm["theta_anat"][cm["valid"]] - cm["theta_ref"][cm["valid"]]) ** 2).mean())
check("theta_anat (giải phẫu) khác theta_ref khi có lệch lắp đặt (nhiễu chéo trục) > 0,5 độ", ana > 0.5, f"{ana:.2f}°")
check("giảm hệ số kết hợp STA xuống 0,5 làm RMS delta giảm", r_off < r_half < r_on, f"{r_half:.2f}°")

# 8) nhãn --------------------------------------------------------------------------------------------------------------------
v = s["valid"]
check("150 mẫu đầu (hiệu chuẩn) bị đánh dấu không hợp lệ, phần còn lại hợp lệ", (~v[:150]).all() and v[150:].all())
check("delta = theta_ref - theta_base ở mọi mẫu hợp lệ", np.allclose(s["delta"][v], (s["theta_ref"][v].astype(np.float64) - s["theta_base"][v]), atol=1e-4))
check("theta_ref bằng 0 (tare) trong lúc hiệu chuẩn", np.abs(s["theta_ref"][:150]).max() < 1e-3)
dts = np.diff(s["ts_us"][:165]); check("không rơi mẫu trong 160 mẫu đầu, jitter timestamp <= 600 us", dts.min() > 19300 and dts.max() < 20700, f"{dts.min()}..{dts.max()}")
check("gyro không bị kẹp ở ±250 °/s (n_clipped = 0)", s["meta"]["n_clipped"] == 0 and s["meta"]["max_gyro_dps"] < 250, f"max {s['meta']['max_gyro_dps']:.0f} °/s")

# 9) tái lập ----------------------------------------------------------------------------------------------------------------------
a1 = simulate_sequence(sub, P, seed=21, seconds=20); a2 = simulate_sequence(sub, P, seed=21, seconds=20); a3 = simulate_sequence(sub, P, seed=22, seconds=20)
check("cùng seed -> dữ liệu giống hệt", all(np.array_equal(a1[k], a2[k]) for k in ("a1", "g2", "theta_ref", "delta", "ts_us") if k != "delta") and np.array_equal(a1["delta"], a2["delta"], equal_nan=True))
check("khác seed -> dữ liệu khác", not np.array_equal(a1["g2"], a3["g2"]))
b_on, b_off = simulate_sequence(sub, P, seed=21, seconds=20), simulate_sequence(sub, P, seed=21, seconds=20, sta_on=False)
check("bật/tắt STA không làm đổi quỹ đạo, timestamp (dòng ngẫu nhiên tách riêng)", np.array_equal(b_on["theta_ref"], b_off["theta_ref"]) and np.array_equal(b_on["ts_us"], b_off["ts_us"]))

# 10) ngân hàng quỹ đạo thật ---------------------------------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as d:
    t = np.arange(0, 20, 0.02)
    np.savetxt(f"{d}/rec.csv", np.column_stack([t, 80 * np.sin(2 * np.pi * 0.2 * t), 60 * np.sin(2 * np.pi * 0.15 * t)]), delimiter=",", header="t_s,pitch_deg,roll_deg", comments="")
    pb = Params(); pb.motion.p_bank = 1.0
    bk = TrajectoryBank(d)
    q = simulate_sequence(sub, pb, seed=2, seconds=40, bank=bk)
    check("ngân hàng quỹ đạo: đọc được và quỹ đạo sinh ra vẫn nằm trong dải sinh lý",
          len(bk) == 1 and q["rel_pitch_true"].min() >= pb.physio.pitch_deg[0] - 1e-3 and q["rel_pitch_true"].max() <= pb.physio.pitch_deg[1] + 1e-3
          and np.abs(q["rel_roll_true"]).max() <= pb.physio.roll_deg[1] + 1e-3, f"pitch {q['rel_pitch_true'].min():.1f}..{q['rel_pitch_true'].max():.1f}")

# 11) đóng gói tập dữ liệu và chia theo người ----------------------------------------------------------------------------------
with tempfile.TemporaryDirectory() as d:
    rows = make_dataset(d, P, n_subjects=3, seq_per_subject=2, seconds=30, seed=1, paired_control=True)
    z = np.load(f"{d}/{rows[0]['file']}", allow_pickle=False)
    need = {"ts_us", "a1", "g1", "a2", "g2", "theta_ref", "theta_base", "delta", "valid", "theta_base_noSTA", "sta_head_deg", "meta"}
    check("tệp .npz đủ trường (kể cả cặp đối chứng không STA)", need <= set(z.files))
    sp = loso_splits(f"{d}/manifest.csv")
    allf = {r["file"] for r in rows}
    ok = len(sp) == 3 and all(set(tr_) | set(te) == allf and not (set(tr_) & set(te)) for _, tr_, te in sp)
    ok = ok and all(all(r["subject"] != s_ for r in rows if r["file"] in tr_) for s_, tr_, te in sp)
    check("LOSO: mỗi lần giữ nguyên một người, không rò rỉ người giữa train và test", ok)
    check("có params.json và dataset_meta.json đi kèm", os.path.exists(f"{d}/params.json") and os.path.exists(f"{d}/dataset_meta.json"))

# 12) nhãn bằng mã C trùng nhãn bằng golden Python ------------------------------------------------------------------------------
subprocess.check_call(["make", "-C", f"{ROOT}/c/host_test", "-s", "replay"])
cc = simulate_sequence(sub, P, seed=21, seconds=20, label_backend="c", replay_exe=f"{ROOT}/c/host_test/replay")
d_ = np.nanmax(np.abs(cc["theta_base"].astype(np.float64) - a1["theta_base"].astype(np.float64)))
check("nhãn gán bằng mã C trùng nhãn golden Python (< 1e-3 độ)", d_ < 1e-3, f"{d_:.1e}")

print("TẤT CẢ ĐẠT" if all(checks) else "CÓ LỖI"); sys.exit(0 if all(checks) else 1)
