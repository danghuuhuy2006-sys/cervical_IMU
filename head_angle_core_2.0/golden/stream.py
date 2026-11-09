"""Luồng từng mẫu cho cả hệ thống: hiệu chuẩn -> warm-up -> chạy -> góc tương đối + LERP.

API khớp mã C (hap_stream_*):
    s = HeadAngleStream(cfg)
    s.push_calib(sample)    # gọi calib_samples lần
    while not s.warmup_round(): pass
    out = s.step(sample)    # mỗi mẫu sau đó
    s.tare()                # phím Z của Giai đoạn 1
"""
from .config import Config, F
from .types import Output
from .sensor import SensorPipeline
from .lerp import CausalLerp
from .quat import q_inv, q_mul, relative_euler
from .calib import mean_std3, norm4_diff

COLLECTING, WARMUP, RUNNING = 0, 1, 2


class HeadAngleStream:
    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self.s1 = SensorPipeline(self.cfg)    # T1-T3 (nền)
        self.s2 = SensorPipeline(self.cfg)    # đầu
        self.lp = CausalLerp(self.cfg.lerp_alpha)
        self.lr = CausalLerp(self.cfg.lerp_alpha)
        self.state = COLLECTING
        self.buf = []
        self.rounds = 0
        self.report = {}
        self.qr1 = self.qr2 = None

    # ---- hiệu chuẩn -------------------------------------------------------
    def push_calib(self, x):
        assert self.state == COLLECTING
        self.buf.append(x)
        if len(self.buf) < self.cfg.calib_samples:
            return self.state
        m1, sd1 = mean_std3([b.g1 for b in self.buf])
        m2, sd2 = mean_std3([b.g2 for b in self.buf])
        self.s1.gyro_bias = list(m1)
        self.s2.gyro_bias = list(m2)
        warn = any(v > self.cfg.bias_std_warn for v in list(sd1) + list(sd2))
        self.report = dict(bias1=m1, bias2=m2, std1=sd1, std2=sd2, vibration_warning=warn,
                           rounds=0, converged=False, dq1=F(0), dq2=F(0))
        self.s1.reset()
        self.s2.reset()
        self.rounds = 0
        self.state = WARMUP
        return self.state

    def warmup_round(self):
        """Chạy một vòng warm-up. Trả True khi kết thúc (hội tụ hoặc hết số vòng)."""
        assert self.state == WARMUP
        p1, p2 = self.s1.get_q(), self.s2.get_q()
        for v in self.buf:
            self.s1.process(*v.g1, *v.a1, v.ts_us)
            self.s2.process(*v.g2, *v.a2, v.ts_us)
        d1 = norm4_diff(self.s1.get_q(), p1)
        d2 = norm4_diff(self.s2.get_q(), p2)
        self.rounds += 1
        conv = bool(d1 < self.cfg.warmup_tol and d2 < self.cfg.warmup_tol)
        self.report.update(rounds=self.rounds, converged=conv, dq1=d1, dq2=d2)
        if conv or self.rounds >= self.cfg.warmup_max_rounds:
            self.qr1, self.qr2 = self.s1.get_q(), self.s2.get_q()
            self.lp.reset(0.0)
            self.lr.reset(0.0)
            self.state = RUNNING
            return True
        return False

    def calibrate(self, samples):
        """Tiện ích cho mô phỏng: nạp đủ mẫu và chạy hết warm-up."""
        for x in samples[: self.cfg.calib_samples]:
            self.push_calib(x)
        while not self.warmup_round():
            pass
        return self.report

    # ---- chạy -------------------------------------------------------------
    def step(self, x):
        assert self.state == RUNNING
        q1 = self.s1.process(*x.g1, *x.a1, x.ts_us)
        q2 = self.s2.process(*x.g2, *x.a2, x.ts_us)
        q1_adj = q_mul(q_inv(self.qr1), q1)
        q2_adj = q_mul(q_inv(self.qr2), q2)
        rp, rr, ry = relative_euler(q1_adj, q2_adj)
        sp = self.lp.step(rp)
        sr = self.lr.step(rr)
        return Output(rp, rr, sp, sr, q1, q2,
                      self.s1.madgwick.beta, self.s2.madgwick.beta,
                      self.s1.zupt, self.s2.zupt, ry)

    def tare(self):
        self.qr1, self.qr2 = self.s1.get_q(), self.s2.get_q()
