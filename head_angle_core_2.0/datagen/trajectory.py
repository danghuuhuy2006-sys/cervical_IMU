"""Bộ sinh quỹ đạo đầu/thân sạch (bước 1 đến 3): ngân hàng quỹ đạo thật + bộ sinh tham số, có ràng buộc dải sinh lý."""
import glob
import os
import numpy as np


def sta_angle_deg(d_mm, r_m):
    """Quy đổi dịch chuyển da d (mm) sang góc (độ) bằng d / r (gần đúng, r = cánh tay đòn tới tâm quay)."""
    return float(np.degrees((d_mm * 1e-3) / r_m))


def _minjerk(tau):
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def gen_axis_track(rng, n, fs, lim, motion, physio, start_idx, v_peak_max=None):
    """Quỹ đạo một trục (độ). Đứng yên tới start_idx, sau đó chuỗi đoạn min-jerk + dừng ngẫu nhiên.
    Ràng buộc: mọi đích nằm trong [lim[0], lim[1]]; tốc độ đỉnh <= v_peak_max."""
    lo, hi = lim
    vmax = physio.v_peak_max_dps if v_peak_max is None else v_peak_max
    out = np.zeros(n)
    i, cur = start_idx, 0.0
    while i < n:
        if rng.random() < physio.p_extreme:                      # đích gần biên để bao phủ dải
            tgt = (hi if rng.random() < 0.5 else lo) * rng.uniform(0.85, 1.0)
        else:
            tgt = rng.uniform(lo, hi)
        amp = abs(tgt - cur)
        if amp >= motion.min_amp_deg:
            v_peak = np.exp(rng.uniform(np.log(motion.v_peak_min_dps), np.log(vmax)))
            dur = max(1.875 * amp / v_peak, motion.t_min_s)       # đỉnh vận tốc min-jerk = 1,875 A / T
            if 1.875 * amp / dur > vmax:                          # bảo đảm không vượt tốc độ tối đa
                dur = 1.875 * amp / vmax
            m = max(2, int(round(dur * fs)))
            seg = cur + (tgt - cur) * _minjerk(np.linspace(0, 1, m + 1)[1:])
            j = min(n, i + m)
            out[i:j] = seg[: j - i]
            cur = tgt if i + m <= n else seg[j - i - 1]
            i += m
        pause = rng.uniform(*motion.long_pause_s) if rng.random() < motion.p_long_pause else rng.uniform(*motion.pause_s)
        j = min(n, i + int(round(pause * fs)))
        out[i:j] = cur
        i = j
    return np.clip(out, lo, hi)


def trunk_track(rng, n, fs, motion, start_idx):
    """Thân: nhịp thở + dao động chậm, bắt đầu từ 0 sau thời gian đứng yên. Trả (pitch, roll) độ."""
    t = np.arange(n) / fs
    ramp = np.clip((np.arange(n) - start_idx) / (1.5 * fs), 0, 1)
    out = []
    for _ in range(2):
        b = rng.uniform(*motion.breath_amp_deg) * np.sin(2 * np.pi * rng.uniform(*motion.breath_hz) * t + rng.uniform(0, 6.28))
        s = rng.uniform(*motion.sway_amp_deg) * np.sin(2 * np.pi * rng.uniform(*motion.sway_hz) * t + rng.uniform(0, 6.28))
        out.append(ramp * (b * 0.5 + s))
    return out[0], out[1]


class TrajectoryBank:
    """Quỹ đạo thật (Giai đoạn 1, giàn thử, in-vivo). Mỗi tệp CSV: cột t_s, pitch_deg, roll_deg (roll tùy chọn)."""

    def __init__(self, directory=None):
        self.items = []
        if directory and os.path.isdir(directory):
            for f in sorted(glob.glob(os.path.join(directory, "*.csv"))):
                d = np.genfromtxt(f, delimiter=",", names=True)
                names = d.dtype.names
                if "t_s" in names and "pitch_deg" in names:
                    roll = d["roll_deg"] if "roll_deg" in names else np.zeros(len(d))
                    self.items.append((os.path.basename(f), d["t_s"], d["pitch_deg"], roll))

    def __len__(self):
        return len(self.items)

    def sample(self, rng, n, fs, lim_p, lim_r, motion, start_idx):
        """Lấy một quỹ đạo thật rồi ngẫu nhiên hóa: co giãn biên độ, co giãn thời gian, chèn dừng; kẹp theo dải sinh lý."""
        _, t, p, r = self.items[rng.integers(len(self.items))]
        warp = rng.uniform(0.7, 1.4)
        amp = rng.uniform(0.7, 1.1)
        tt = np.arange(0, n - start_idx) / fs
        src_t = (t - t[0]) * warp
        reps = int(np.ceil(tt[-1] / max(src_t[-1], 1e-6))) + 1
        src_t = np.concatenate([src_t + k * (src_t[-1] + 1.0) for k in range(reps)])
        pp = np.tile(np.concatenate([p, [p[-1]]])[: len(t)], reps)
        rr = np.tile(np.concatenate([r, [r[-1]]])[: len(t)], reps)
        # chèn dừng ngẫu nhiên bằng cách giữ giá trị trong vài khoảng
        hold = np.zeros(len(tt), bool)
        for _ in range(int(len(tt) / fs / 8)):
            a = rng.integers(0, len(tt)); hold[a:a + int(rng.uniform(*motion.pause_s) * fs)] = True
        P = np.interp(tt, src_t[: len(pp)], pp) * amp
        R = np.interp(tt, src_t[: len(rr)], rr) * amp
        for arr in (P, R):
            idx = np.where(hold)[0]
            for k in idx:
                if k > 0:
                    arr[k] = arr[k - 1]
        pitch = np.zeros(n); roll = np.zeros(n)
        pitch[start_idx:] = np.clip(P - P[0], *lim_p)
        roll[start_idx:] = np.clip(R - R[0], *lim_r)
        return pitch, roll


def physio_report(tracks, physio):
    """Báo cáo bao phủ và vi phạm dải sinh lý của một tập quỹ đạo (độ)."""
    rep = {}
    for name, lim in (("pitch", physio.pitch_deg), ("roll", tuple(np.array(physio.roll_deg) * 1.0))):
        x = np.concatenate([t[name] for t in tracks])
        rep[name] = dict(min=float(x.min()), max=float(x.max()), lim=lim,
                         violations=int(((x < lim[0] - 1e-9) | (x > lim[1] + 1e-9)).sum()),
                         coverage_lo=float(x.min() / lim[0]) if lim[0] else 0.0,
                         coverage_hi=float(x.max() / lim[1]) if lim[1] else 0.0)
    return rep
