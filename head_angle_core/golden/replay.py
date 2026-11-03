"""Phát lại một tệp CSV số (seq,ts_us,a1x,a1y,a1z,g1x,g1y,g1z,a2x,...,g2z) qua golden float32.

    python -m golden.replay in.csv out.csv
Đầu ra: idx,ts_us,pitch_raw,roll_raw,pitch_smooth,roll_smooth,beta1,beta2,zupt1,zupt2
Dòng đầu của out.csv (chú thích #) ghi báo cáo hiệu chuẩn.
"""
import sys
import numpy as np
from .config import Config
from .types import Sample
from .stream import HeadAngleStream


def load_samples(path):
    rows = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float64)
    out = []
    for r in rows:
        out.append(Sample(ts_us=int(r[1]), a1=tuple(r[2:5]), g1=tuple(r[5:8]),
                          a2=tuple(r[8:11]), g2=tuple(r[11:14])))
    return out


def main(inp, outp):
    samples = load_samples(inp)
    cfg = Config()
    s = HeadAngleStream(cfg)
    rep = s.calibrate(samples)
    with open(outp, "w") as f:
        f.write("# calib rounds=%d converged=%d vib=%d bias1=%s bias2=%s\n" % (
            rep["rounds"], rep["converged"], rep["vibration_warning"],
            " ".join("%.9g" % v for v in rep["bias1"]), " ".join("%.9g" % v for v in rep["bias2"])))
        f.write("idx,ts_us,pitch_raw,roll_raw,pitch_smooth,roll_smooth,beta1,beta2,zupt1,zupt2,yaw_raw\n")
        for i, x in enumerate(samples[cfg.calib_samples:], start=cfg.calib_samples):
            o = s.step(x)
            f.write("%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.9g\n" % (
                i, x.ts_us, o.pitch_raw, o.roll_raw, o.pitch_smooth, o.roll_smooth,
                o.beta1, o.beta2, int(o.zupt1), int(o.zupt2), o.yaw_raw))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
