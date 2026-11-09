"""Mô phỏng IMU + STA, gán nhãn bằng golden model, đóng gói tập dữ liệu và chia theo người."""
import csv
import hashlib
import json
import os
import subprocess
import tempfile
from dataclasses import dataclass, asdict

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

from . import quat as Q
from .params import Params
from .trajectory import gen_axis_track, trunk_track, TrajectoryBank

G0 = 9.80665
CALIB_N = 150          # golden hiệu chuẩn bằng 150 mẫu đầu


@dataclass
class Subject:
    sid: str
    group: str
    thickness_mm: float
    sta_scale: float
    mount1: list       # vector quay (rad) cảm biến 1 (thân) so với xương
    mount2: list       # cảm biến 2 (đầu)
    bias1: list
    bias2: list
    acc_off1: list
    acc_off2: list
    acc_scale1: list
    acc_scale2: list


def sample_subject(rng, sid, group, params=None):
    p = params or Params()
    lo, hi = p.sta.thickness_mm[group]
    thick = float(rng.uniform(lo, hi))
    scale = thick / p.sta.thickness_ref_mm * float(np.exp(rng.normal(0.0, p.sta.individual_logsd)))
    msd = np.radians(p.noise.mount_sd_deg)
    v = lambda sd: [float(x) for x in rng.normal(0.0, sd, 3)]
    return Subject(sid, group, thick, scale, v(msd), v(msd), v(p.noise.gyro_bias_sd_rads), v(p.noise.gyro_bias_sd_rads),
                   v(p.noise.accel_offset_sd_g), v(p.noise.accel_offset_sd_g),
                   [1.0 + x for x in v(p.noise.accel_scale_sd)], [1.0 + x for x in v(p.noise.accel_scale_sd)])


def _lowpass_unit(rng, n, fs, fc, shape):
    """Nhiễu Gauss qua bộ lọc thông thấp bậc 2, chuẩn hóa độ lệch chuẩn = 1 trên toàn chuỗi."""
    sos = butter(2, fc, fs=fs, output="sos")
    w = rng.standard_normal((n,) + shape)
    x = sosfilt(sos, w, axis=0)
    return x / (x.std(axis=0, keepdims=True) + 1e-12)


def _sta_sigma_mm(scale, dmax_mm, g, p):
    return p.sta.sigma0_mm + scale * dmax_mm * p.sta.sigma_over_dmax * g


def simulate_sequence(subject, params=None, seed=0, seconds=120.0, sta_on=True, noise_on=True, mount_on=True,
                      bank=None, label_backend="python", replay_exe=None):
    """Trả dict gồm IMU thô (50 Hz), góc tham chiếu, góc nền từ golden, nhãn và thông tin chẩn đoán."""
    p = params or Params()
    ss = np.random.SeedSequence(seed).spawn(4)
    r_traj, r_sta, r_noise, r_misc = [np.random.default_rng(s) for s in ss]   # tách dòng ngẫu nhiên: bật/tắt STA không đổi quỹ đạo và nhiễu
    fs, fo = p.fs_sim, p.fs_out
    n = int(seconds * fs)
    k0 = int(p.motion.static_s * fs)
    dt = 1.0 / fs
    ph = p.physio

    # --- 1) quỹ đạo sạch: đầu so với thân (độ) và thân ------------------------------------------------
    if bank is not None and len(bank) and r_traj.random() < p.motion.p_bank:
        pitch, roll = bank.sample(r_traj, n, fs, ph.pitch_deg, tuple(np.array(ph.roll_deg) * ph.roll_scale), p.motion, k0)
    else:
        pitch = gen_axis_track(r_traj, n, fs, ph.pitch_deg, p.motion, ph, k0)
        roll = gen_axis_track(r_traj, n, fs, tuple(np.array(ph.roll_deg) * ph.roll_scale), p.motion, ph, k0)
    yaw = gen_axis_track(r_traj, n, fs, tuple(np.array(ph.yaw_deg) * ph.yaw_scale), p.motion, ph, k0)
    bp, br = trunk_track(r_traj, n, fs, p.motion, k0)
    yaw0 = float(r_misc.uniform(-np.pi, np.pi))

    q_base = Q.from_euler(np.radians(br), np.radians(bp), np.full(n, yaw0))
    q_rel = Q.from_euler(np.radians(roll), np.radians(pitch), np.radians(yaw))
    q_head = Q.qnorm(Q.qmul(q_base, q_rel))

    # --- 2) STA: sigma(t) tăng theo biên độ và tốc độ; phép quay chậm + gia tốc tuyến tính giả -------------
    def motion_proxy(pt, rl, lim_p, lim_r):
        a = np.maximum(np.abs(pt) / max(abs(lim_p[0]), abs(lim_p[1])), np.abs(rl) / max(abs(lim_r[0]), abs(lim_r[1])))
        v = np.maximum(np.abs(np.gradient(pt, dt)), np.abs(np.gradient(rl, dt))) / p.sta.v_ref_dps
        return np.clip(p.sta.w_amp * a + p.sta.w_vel * v, 0.0, 1.0)

    dmax = np.array([p.sta.d_max_roll_mm, p.sta.d_max_pitch_mm, p.sta.d_max_pitch_mm])   # quanh x (roll), y (pitch), z (yaw)
    out_sta = {}
    qs = {}
    rigid = {}         # hướng cảm biến gắn CỨNG (có lệch lắp đặt, không STA): "tín hiệu sạch" của Giai đoạn 1
    for name, (qb, pt, rl, scale, lever, mount) in {
        "head": (q_head, pitch, roll, subject.sta_scale, p.sta.lever_head_m, subject.mount2),
        "base": (q_base, bp, br, subject.sta_scale * p.sta.base_scale, p.sta.lever_base_m, subject.mount1),
    }.items():
        g_t = motion_proxy(pt, rl, ph.pitch_deg, ph.roll_deg)
        sig_mm = _sta_sigma_mm(scale, dmax[None, :], g_t[:, None], p)                # (n,3)
        nu = _lowpass_unit(r_sta, n, fs, p.sta.fc_hz, (3,))
        phi = (sig_mm * 1e-3 / lever) * p.sta.tilt_coupling * nu                      # d / r (rad)
        npos = _lowpass_unit(r_sta, n, fs, p.sta.fc_hz, (3,))
        pos = (sig_mm[:, :1] * 1e-3) * npos * np.array(p.sta.lin_axes_scale)          # dịch chuyển da (m) trong hệ cảm biến
        alin = np.gradient(np.gradient(pos, dt, axis=0), dt, axis=0) / G0             # g
        q = qb
        if mount_on:
            q = Q.qmul(q, Q.from_rotvec(np.array(mount))[None, :])
        rigid[name] = Q.qnorm(q)
        if sta_on:
            q = Q.qmul(q, Q.from_rotvec(phi))
        else:
            phi = np.zeros_like(phi); alin = np.zeros_like(alin)
        qs[name] = (Q.qnorm(q), alin)
        out_sta[name] = np.degrees(phi)

    # --- 3) IMU lý tưởng -> DLPF -> hạ mẫu --------------------------------------------------------------
    sos = butter(2, p.noise.dlpf_hz, fs=fs, output="sos")
    zi0 = sosfilt_zi(sos)[:, :, None]

    def dlpf(x):
        return sosfilt(sos, x, axis=0, zi=zi0 * x[0])[0]

    step = fs // fo
    off = int(r_misc.integers(0, step))
    sel = np.arange(off, n, step)
    raw = {}
    for name in ("base", "head"):
        q, alin = qs[name]
        raw[name] = (dlpf(Q.body_rate(q, dt))[sel], dlpf(Q.gravity_in_body(q) + alin)[sel])
    N = len(sel)

    # --- 4) nhiễu cảm biến, hiệu chuẩn lệch, lượng tử hóa, timestamp --------------------------------------
    n_clip = 0
    sensors = {}
    for name, bias, aoff, ascale in (("base", subject.bias1, subject.acc_off1, subject.acc_scale1),
                                     ("head", subject.bias2, subject.acc_off2, subject.acc_scale2)):
        g, a = raw[name][0].copy(), raw[name][1].copy()
        if noise_on:
            walk = np.cumsum(r_noise.normal(0.0, p.noise.gyro_walk_rads_sqrts * np.sqrt(1.0 / fo), (N, 3)), axis=0)
            g = g + np.array(bias) + walk + r_noise.normal(0.0, p.noise.gyro_white_rads, (N, 3))
            a = a * np.array(ascale) + np.array(aoff) + r_noise.normal(0.0, p.noise.accel_white_g, (N, 3))
            gr = np.radians(p.noise.gyro_range_dps)
            n_clip += int((np.abs(g) > gr).sum() + (np.abs(a) > p.noise.accel_range_g).sum())
            g = np.clip(g, -gr, gr); a = np.clip(a, -p.noise.accel_range_g, p.noise.accel_range_g)
            lsb_g = p.noise.gyro_lsb_per_dps * 180.0 / np.pi
            g = np.round(g * lsb_g) / lsb_g
            a = np.round(a * p.noise.accel_lsb_per_g) / p.noise.accel_lsb_per_g
        sensors[name] = (g, a)

    t0 = int(r_misc.integers(1_000_000, 1_000_000_000))
    jit = r_noise.integers(-p.noise.jitter_us, p.noise.jitter_us + 1, N) if noise_on else np.zeros(N, int)
    ts = np.maximum.accumulate(t0 + np.arange(N) * (1_000_000 // fo) + jit).astype(np.int64)
    keep = np.ones(N, bool)
    if noise_on and p.noise.drop_prob > 0:
        keep[CALIB_N + 10:] = r_noise.random(N - CALIB_N - 10) >= p.noise.drop_prob     # không rơi mẫu trong/ngay sau hiệu chuẩn

    # --- 5) góc tham chiếu, tare tại cuối hiệu chuẩn (mẫu 149) ----------------------------------------------------------
    #   theta_ref  : cảm biến gắn cứng (có lệch lắp đặt, KHÔNG STA) = tín hiệu sạch kiểu Giai đoạn 1 -> nhãn tách riêng STA
    #   theta_anat : góc xương trong hệ giải phẫu (không lệch lắp đặt, không STA), chỉ để phân tích
    i0 = CALIB_N - 1

    def tared_angles(qb_full, qh_full):
        qb_o, qh_o = qb_full[sel], qh_full[sel]
        pp, rr = Q.relative_pitch_roll_deg(Q.qmul(Q.qconj(qb_o[i0])[None, :], qb_o), Q.qmul(Q.qconj(qh_o[i0])[None, :], qh_o))
        return np.stack([pp, rr], axis=1)

    theta_ref = tared_angles(rigid["base"], rigid["head"])
    theta_anat = tared_angles(q_base, q_head)

    seq = {
        "ts_us": ts[keep], "a1": sensors["base"][1][keep].astype(np.float32), "g1": sensors["base"][0][keep].astype(np.float32),
        "a2": sensors["head"][1][keep].astype(np.float32), "g2": sensors["head"][0][keep].astype(np.float32),
        "theta_ref": theta_ref[keep].astype(np.float32), "theta_anat": theta_anat[keep].astype(np.float32),
        "sta_head_deg": out_sta["head"][sel][keep].astype(np.float32), "sta_base_deg": out_sta["base"][sel][keep].astype(np.float32),
        "rel_pitch_true": pitch[sel][keep].astype(np.float32), "rel_roll_true": roll[sel][keep].astype(np.float32),
    }
    base = label_with_golden(seq, label_backend, replay_exe)
    seq["theta_base"] = base.astype(np.float32)
    seq["delta"] = (seq["theta_ref"].astype(np.float64) - base).astype(np.float32)
    seq["valid"] = np.isfinite(base).all(axis=1)
    rate = np.degrees(np.abs(np.concatenate([seq["g1"], seq["g2"]]))).max()
    seq["meta"] = dict(subject=subject.sid, group=subject.group, thickness_mm=subject.thickness_mm, sta_scale=subject.sta_scale,
                       seed=int(seed), seconds=float(seconds), sta_on=bool(sta_on), noise_on=bool(noise_on), mount_on=bool(mount_on),
                       n_samples=int(len(seq["ts_us"])), n_valid=int(seq["valid"].sum()), n_clipped=n_clip, max_gyro_dps=float(rate),
                       params_sha256=params_hash(p))
    return seq


def params_hash(p):
    return hashlib.sha256(p.to_json().encode()).hexdigest()[:16]


def label_with_golden(seq, backend="python", replay_exe=None):
    """Chạy luồng thô qua golden (float32) hoặc mã C để lấy theta_base (pitch_raw, roll_raw). NaN cho 150 mẫu hiệu chuẩn."""
    N = len(seq["ts_us"])
    out = np.full((N, 2), np.nan)
    if backend == "python":
        from golden.config import F
        if F is not np.float32:
            raise RuntimeError("golden đang ở chế độ float64; gán nhãn phải dùng float32 (bỏ HAP_DTYPE)")
        from golden import HeadAngleStream, Sample
        s = HeadAngleStream()
        for i in range(N):
            x = Sample(int(seq["ts_us"][i]), tuple(seq["a1"][i]), tuple(seq["g1"][i]), tuple(seq["a2"][i]), tuple(seq["g2"][i]))
            if i < s.cfg.calib_samples:
                s.push_calib(x)
                if i == s.cfg.calib_samples - 1:
                    while not s.warmup_round():
                        pass
            else:
                o = s.step(x)
                out[i] = (float(o.pitch_raw), float(o.roll_raw))
    elif backend == "c":
        with tempfile.TemporaryDirectory() as d:
            fi, fo = os.path.join(d, "in.csv"), os.path.join(d, "out.csv")
            with open(fi, "w") as f:
                f.write("seq,ts_us,a1x,a1y,a1z,g1x,g1y,g1z,a2x,a2y,a2z,g2x,g2y,g2z\n")
                for i in range(N):
                    v = [seq["a1"][i], seq["g1"][i], seq["a2"][i], seq["g2"][i]]
                    f.write("%d,%d,%s\n" % (i, seq["ts_us"][i], ",".join("%.9g" % x for a in v for x in a)))
            subprocess.check_call([replay_exe, fi, fo])
            arr = np.loadtxt(fo, delimiter=",", skiprows=2)
            out[arr[:, 0].astype(int), 0] = arr[:, 2]
            out[arr[:, 0].astype(int), 1] = arr[:, 3]
    else:
        raise ValueError(backend)
    return out


def save_sequence(path, seq):
    meta = seq["meta"]
    arrays = {k: v for k, v in seq.items() if k != "meta"}
    np.savez_compressed(path, meta=json.dumps(meta), **arrays)


def make_dataset(out_dir, params=None, n_subjects=9, seq_per_subject=2, seconds=120.0, seed=0,
                 label_backend="python", replay_exe=None, bank_dir=None, paired_control=False):
    """Sinh cả tập dữ liệu. Mỗi 'người' có nhân trắc, lệch lắp đặt, bias riêng và nhiều phiên; chia tập theo người."""
    p = params or Params()
    os.makedirs(out_dir, exist_ok=True)
    bank = TrajectoryBank(bank_dir)
    groups = ["low", "mid", "high"]
    rng = np.random.default_rng(seed)
    rows = []
    for si in range(n_subjects):
        sub = sample_subject(rng, "S%03d" % si, groups[si % 3], p)
        for k in range(seq_per_subject):
            sseed = int(rng.integers(0, 2**31 - 1))
            seq = simulate_sequence(sub, p, sseed, seconds, bank=bank, label_backend=label_backend, replay_exe=replay_exe)
            if paired_control:     # cùng quỹ đạo và nhiễu nhưng không có STA, để tách phần do STA
                ctl = simulate_sequence(sub, p, sseed, seconds, sta_on=False, bank=bank, label_backend=label_backend, replay_exe=replay_exe)
                seq["theta_base_noSTA"] = ctl["theta_base"]
            fn = "%s_%02d.npz" % (sub.sid, k)
            save_sequence(os.path.join(out_dir, fn), seq)
            m = seq["meta"]
            rows.append(dict(file=fn, subject=sub.sid, group=sub.group, thickness_mm="%.2f" % sub.thickness_mm,
                             sta_scale="%.3f" % sub.sta_scale, seconds=seconds, n_valid=m["n_valid"], seed=sseed))
    with open(os.path.join(out_dir, "manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    p.to_json(os.path.join(out_dir, "params.json"))
    json.dump(dict(seed=seed, n_subjects=n_subjects, seq_per_subject=seq_per_subject, seconds=seconds,
                   label_backend=label_backend, params_sha256=params_hash(p), bank_size=len(bank),
                   NOTE="Tham số nhiễu/STA mặc định là giả định (dấu † trong params.py); thay bằng Allan và số đo người thật trước khi huấn luyện thật."),
              open(os.path.join(out_dir, "dataset_meta.json"), "w"), indent=2, ensure_ascii=False)
    return rows


def loso_splits(manifest):
    """Leave-one-subject-out: mỗi lần giữ lại toàn bộ phiên của một người làm tập kiểm tra. manifest: đường dẫn CSV hoặc list dict."""
    if isinstance(manifest, str):
        manifest = list(csv.DictReader(open(manifest)))
    subs = sorted({r["subject"] for r in manifest})
    out = []
    for s in subs:
        out.append((s, [r["file"] for r in manifest if r["subject"] != s], [r["file"] for r in manifest if r["subject"] == s]))
    return out
