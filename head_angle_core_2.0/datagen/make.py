"""python -m datagen.make --out data/syn --subjects 9 --seq 2 --seconds 120 --seed 0 [--backend c] [--bank dir] [--paired]"""
import argparse, os, sys
from .dataset import make_dataset
from .params import Params

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--subjects", type=int, default=9)
ap.add_argument("--seq", type=int, default=2, help="số phiên mỗi người")
ap.add_argument("--seconds", type=float, default=120.0)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--backend", choices=["python", "c"], default="python")
ap.add_argument("--replay", default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "c/host_test/replay"))
ap.add_argument("--bank", help="thư mục CSV quỹ đạo thật (t_s,pitch_deg,roll_deg)")
ap.add_argument("--params", help="tệp params.json để ghi đè mặc định")
ap.add_argument("--paired", action="store_true", help="thêm theta_base không STA cho cùng quỹ đạo (gấp đôi thời gian)")
a = ap.parse_args()
p = Params.from_json(a.params) if a.params else Params()
rows = make_dataset(a.out, p, a.subjects, a.seq, a.seconds, a.seed, a.backend, a.replay, a.bank, a.paired)
print(f"đã ghi {len(rows)} phiên vào {a.out}")
