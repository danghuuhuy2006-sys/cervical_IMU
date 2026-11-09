"""API so khớp số (Numerical Matching) giữa golden model Python và firmware.

Đầu vào hai tệp CSV cùng định dạng với golden.replay / c/host_test/replay:
    idx,ts_us,pitch_raw,roll_raw,pitch_smooth,roll_smooth,beta1,beta2,zupt1,zupt2,yaw_raw
(dòng bắt đầu bằng '#' là chú thích). Firmware trên chip chỉ cần ghi log theo đúng cột này.

Quy tắc FAIL (mặc định ngưỡng 0,05 độ cho các kênh góc):
  - số dòng khác nhau, hoặc idx / ts_us không trùng nhau;
  - có NaN hoặc vô cùng trong đầu ra firmware;
  - sai khác tuyệt đối lớn nhất của bất kỳ kênh nào VƯỢT ngưỡng (bằng ngưỡng thì đạt).

Dùng trong mã:    report = compare(golden_csv, fw_csv, tol=0.05); report.ok
                  assert_match(golden_csv, fw_csv)               # ném NumericalMismatch khi fail
Dùng dòng lệnh:   python -m golden.compare golden.csv fw.csv [--tol 0.05]       (thoát mã 1 nếu fail)
Từ dữ liệu thô:   match_inputs(input_csv, firmware_runner, tol=0.05)             (tự chạy golden và firmware)
"""
import argparse
import os
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field

import numpy as np

ANGLE_CHANNELS = ("pitch_raw", "roll_raw", "pitch_smooth", "roll_smooth")
DEFAULT_TOL = 0.05                       # độ
OPTIONAL_CHANNELS = {"yaw_raw": DEFAULT_TOL}   # chỉ so khi cả hai tệp có cột này và được yêu cầu


class NumericalMismatch(AssertionError):
    pass


@dataclass
class ChannelResult:
    name: str
    tol: float
    max_abs: float
    rms: float
    worst_idx: int
    n_exceed: int

    @property
    def ok(self):
        return self.n_exceed == 0


@dataclass
class MatchReport:
    ok: bool
    n_rows: int
    tol: float
    channels: list = field(default_factory=list)
    problems: list = field(default_factory=list)

    def to_text(self):
        lines = [f"{'ĐẠT' if self.ok else 'KHÔNG ĐẠT'}: {self.n_rows} dòng, ngưỡng {self.tol:g}"]
        for c in self.channels:
            lines.append(f"  {c.name:13s} max|Δ| = {c.max_abs:.3e} (tại idx {c.worst_idx}), RMS = {c.rms:.3e}, "
                         f"vượt ngưỡng {c.tol:g}: {c.n_exceed} dòng {'OK' if c.ok else 'FAIL'}")
        lines += [f"  VẤN ĐỀ: {p}" for p in self.problems]
        return "\n".join(lines)


def _load(path):
    with open(path) as f:
        header = None
        rows = []
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if header is None:
                header = line.split(",")
                continue
            rows.append(line.split(","))
    if header is None:
        raise ValueError(f"{path}: không có tiêu đề")
    data = np.array(rows, dtype=np.float64) if rows else np.zeros((0, len(header)))
    return {h: data[:, i] for i, h in enumerate(header)}


def compare(golden_csv, firmware_csv, tol=DEFAULT_TOL, channels=ANGLE_CHANNELS, include_yaw=False):
    """So khớp hai tệp. tol: số, hoặc dict {kênh: ngưỡng}. Trả MatchReport (không ném lỗi)."""
    g, w = _load(golden_csv), _load(firmware_csv)
    names = list(channels) + (["yaw_raw"] if include_yaw else [])
    tols = {n: (tol[n] if isinstance(tol, dict) else float(tol)) for n in names}
    rep = MatchReport(ok=True, n_rows=len(g.get("idx", [])), tol=min(tols.values()))

    for n in names:
        for src, d in (("golden", g), ("firmware", w)):
            if n not in d:
                rep.problems.append(f"thiếu cột '{n}' trong tệp {src}")
    if "idx" not in g or "idx" not in w:
        rep.problems.append("thiếu cột idx")
    if rep.problems:
        rep.ok = False
        return rep
    if len(g["idx"]) != len(w["idx"]):
        rep.problems.append(f"số dòng khác nhau: golden {len(g['idx'])}, firmware {len(w['idx'])}")
        rep.ok = False
        return rep
    if not np.array_equal(g["idx"], w["idx"]):
        rep.problems.append("cột idx không trùng nhau (hai bản không xử lý cùng chuỗi mẫu)")
    if "ts_us" in g and "ts_us" in w and not np.array_equal(g["ts_us"], w["ts_us"]):
        rep.problems.append("cột ts_us không trùng nhau (khác đầu vào)")

    for n in names:
        a, b = g[n], w[n]
        if not np.isfinite(b).all():
            rep.problems.append(f"đầu ra firmware có NaN/Inf ở kênh {n}")
        if not np.isfinite(a).all():
            rep.problems.append(f"đầu ra golden có NaN/Inf ở kênh {n}")
        d = np.abs(a - b)
        d = np.where(np.isfinite(d), d, np.inf)
        k = int(np.argmax(d)) if len(d) else 0
        rep.channels.append(ChannelResult(
            n, tols[n], float(d.max()) if len(d) else 0.0,
            float(np.sqrt(np.mean(np.where(np.isfinite(d), d, 0) ** 2))) if len(d) else 0.0,
            int(g["idx"][k]) if len(d) else -1, int((d > tols[n]).sum())))
    rep.ok = not rep.problems and all(c.ok for c in rep.channels)
    return rep


def assert_match(golden_csv, firmware_csv, tol=DEFAULT_TOL, **kw):
    rep = compare(golden_csv, firmware_csv, tol, **kw)
    if not rep.ok:
        raise NumericalMismatch(rep.to_text())
    return rep


def run_golden(input_csv, out_csv):
    """Chạy golden float32 trong tiến trình con để luôn dùng đúng chế độ float32."""
    env = dict(os.environ, HAP_DTYPE="float32")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    subprocess.check_call([sys.executable, "-m", "golden.replay", input_csv, out_csv], cwd=root, env=env)


def host_firmware_runner(replay_exe):
    """Bộ chạy firmware trên máy chủ (mã C biên dịch bằng gcc). Với firmware thật, viết một runner khác
    có cùng chữ ký (input_csv, out_csv): ví dụ phát lại mẫu qua UART rồi ghi log đúng định dạng CSV ở đầu tệp."""
    def run(input_csv, out_csv):
        subprocess.check_call([replay_exe, input_csv, out_csv])
    return run


def match_inputs(input_csv, firmware_runner, tol=DEFAULT_TOL, **kw):
    """Đưa CÙNG một tệp dữ liệu thô vào golden và firmware, rồi so khớp. Ném NumericalMismatch nếu fail."""
    with tempfile.TemporaryDirectory() as d:
        g, w = os.path.join(d, "golden.csv"), os.path.join(d, "firmware.csv")
        run_golden(input_csv, g)
        firmware_runner(input_csv, w)
        return assert_match(g, w, tol, **kw)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("golden_csv")
    ap.add_argument("firmware_csv")
    ap.add_argument("--tol", type=float, default=DEFAULT_TOL, help="ngưỡng tuyệt đối (độ), mặc định 0.05")
    ap.add_argument("--yaw", action="store_true", help="so cả yaw_raw")
    a = ap.parse_args(argv)
    rep = compare(a.golden_csv, a.firmware_csv, a.tol, include_yaw=a.yaw)
    print(rep.to_text())
    return 0 if rep.ok else 1


if __name__ == "__main__":
    sys.exit(main())
