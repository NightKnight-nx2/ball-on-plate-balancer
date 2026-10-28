"""Dış döngü LQR: top konum/hız hatasından hedef plaka eğimi (pitch, roll).

Model (sympy'den doğrusallaştırılmış):  e' = A e + B u,   e = [x-xr, vx-vxr, y-yr, vy-vyr],
u = [pitch, roll].   Kontrol yasası:  u = u_ff - K e
u_ff: referans ivmesini sağlayan ileri besleme eğimi (u_ff = B_acc^-1 * a_ref).
K  : python-control `control.lqr` ile (Bryson kuralına yakın ağırlıklarla).
"""
from __future__ import annotations

from dataclasses import dataclass

import control
import numpy as np

from ball_dynamics import BallParams, linearize


@dataclass
class LQRWeights:
    pos_max: float = 0.04       # m   (Bryson: kabul edilebilir max konum hatası)
    vel_max: float = 0.25       # m/s
    tilt_max: float = 0.05      # rad (~2.9°)


class BallPlateLQR:
    def __init__(self, bp: BallParams, w: LQRWeights | None = None, tilt_limit: float = 0.12):
        self.bp = bp
        self.w = w or LQRWeights()
        self.tilt_limit = tilt_limit
        self.A, self.B = linearize(bp)
        Q = np.diag([1 / self.w.pos_max**2, 1 / self.w.vel_max**2] * 2)
        R = np.diag([1 / self.w.tilt_max**2] * 2)
        self.K, self.S, self.poles = control.lqr(self.A, self.B, Q, R)
        # ivme satırları: [vx' ; vy'] = A[[1,3]] e + B[[1,3]] u
        self._Bacc = self.B[[1, 3], :]            # (2x2), köşegen: [[k,0],[0,-k]]
        self._Binv = np.linalg.inv(self._Bacc)

    def feedforward_tilt(self, a_ref) -> np.ndarray:
        """Referans ivmesi için gereken eğim (doğrusal model; sönüm terimi ihmal)."""
        return self._Binv @ np.asarray(a_ref, dtype=float)

    def compute(self, state, ref):
        """state=[x,vx,y,vy]; ref=(x,vx,y,vy,ax,ay) -> (pitch_cmd, roll_cmd)."""
        e = np.array(
            [state[0] - ref[0], state[1] - ref[1], state[2] - ref[2], state[3] - ref[3]]
        )
        u = self.feedforward_tilt([ref[4], ref[5]]) - (self.K @ e)
        u = np.clip(u, -self.tilt_limit, self.tilt_limit)
        return float(u[0]), float(u[1])


# ---------------------------------------------------------------------------
# Referans yörüngeler
# ---------------------------------------------------------------------------
class StaticRef:
    """Sabit nokta (denge testi)."""

    def __init__(self, x=0.0, y=0.0):
        self.x, self.y = x, y

    def __call__(self, t):
        return (self.x, 0.0, self.y, 0.0, 0.0, 0.0)


class LissajousRef:
    """Sekiz (∞) çizen Lissajous:  x = A sin(wt),  y = B sin(2wt).
    Dönüş: (x, vx, y, vy, ax, ay). t<t_ramp için genlik yumuşak açılır."""

    def __init__(self, ax_amp=0.07, ay_amp=0.05, period=8.0, ramp=2.0):
        self.A, self.B, self.w, self.ramp = ax_amp, ay_amp, 2 * np.pi / period, ramp

    def __call__(self, t):
        w = self.w
        if self.ramp > 0 and t < self.ramp:
            # s(t): 0->1 beşinci derece düzgün geçiş, türevleri ile
            u = t / self.ramp
            s = u**3 * (10 - 15 * u + 6 * u**2)
            sd = (30 * u**2 - 60 * u**3 + 30 * u**4) / self.ramp
            sdd = (60 * u - 180 * u**2 + 120 * u**3) / self.ramp**2
        else:
            s, sd, sdd = 1.0, 0.0, 0.0

        def comb(amp, wn):
            f = amp * np.sin(wn * t)
            fd = amp * wn * np.cos(wn * t)
            fdd = -amp * wn**2 * np.sin(wn * t)
            return s * f, s * fd + sd * f, s * fdd + 2 * sd * fd + sdd * f

        x, vx, ax = comb(self.A, w)
        y, vy, ay = comb(self.B, 2 * w)
        return (x, vx, y, vy, ax, ay)
