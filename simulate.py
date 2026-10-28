"""Tam döngü simülasyonu: top -> LQR -> ters kinematik -> step motorlar -> ileri kinematik -> top.

Kullanım:
    python simulate.py balance            # sabit (referanssız) denge testi
    python simulate.py lissajous          # sekiz yörüngesi takibi
    python simulate.py lissajous --stress # hızlı yörünge + zayıf motor (pull-out göster)
    python simulate.py lissajous --show   # pencerede göster (varsayılan: PNG/GIF kaydet)
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field

import numpy as np

from actuator import StepperMotor, StepperParams
from ball_dynamics import BallParams, rk4_step
from controller import BallPlateLQR, LissajousRef, LQRWeights, StaticRef
from kinematics import PlateGeometry


@dataclass
class SimConfig:
    t_end: float = 20.0
    dt: float = 1e-3                  # fizik / aktüatör adımı
    ctrl_rate: float = 100.0          # Hz, LQR örnekleme (ZOH)
    sensor_delay: float = 0.15        # s, kamera/tahmin gecikmesi (kontrolcü gecikmiş durumu görür)
    x0: tuple = (0.03, 0.0, -0.02, 0.0)   # başlangıç top durumu [x, vx, y, vy]
    ball: BallParams = field(default_factory=BallParams)
    geom: PlateGeometry = field(default_factory=PlateGeometry)
    motor: StepperParams = field(default_factory=StepperParams)
    weights: LQRWeights = field(default_factory=LQRWeights)


@dataclass
class SimResult:
    cfg: SimConfig
    t: np.ndarray
    ball: np.ndarray        # (N,4) x,vx,y,vy
    ref: np.ndarray         # (N,6) x,vx,y,vy,ax,ay
    tilt_cmd: np.ndarray    # (N,2) LQR çıkışı pitch, roll
    tilt: np.ndarray        # (N,2) gerçekleşen pitch, roll (ileri kinematik)
    theta_req: np.ndarray   # (N,4) kontrolcünün istediği motor açısı
    phi: np.ndarray         # (N,4) sürücü komut açısı
    theta: np.ndarray       # (N,4) gerçek rotor açısı
    po_limit: np.ndarray    # (N,4) anlık pull-out limiti
    slip: np.ndarray        # (N,4) kayıp adım ofseti
    twist: np.ndarray       # (N,)  4 bacak düzlemsellik kalıntısı
    sync_events: list       # [(t, motor_idx)]
    fell_off: float | None  # top plakadan düştüyse zaman


def run_simulation(cfg: SimConfig, ref_fn, verbose=True) -> SimResult:
    geom = cfg.geom
    ctrl = BallPlateLQR(cfg.ball, cfg.weights, tilt_limit=geom.max_tilt())
    motors = [StepperMotor(cfg.motor) for _ in range(4)]

    n = int(round(cfg.t_end / cfg.dt)) + 1
    ctrl_every = int(round(1.0 / (cfg.ctrl_rate * cfg.dt)))
    dn = int(round(cfg.sensor_delay / cfg.dt))
    t_arr = np.arange(n) * cfg.dt
    ball = np.zeros((n, 4)); ref = np.zeros((n, 6))
    tilt_cmd = np.zeros((n, 2)); tilt = np.zeros((n, 2))
    th_req = np.zeros((n, 4)); phi = np.zeros((n, 4)); th = np.zeros((n, 4))
    po = np.zeros((n, 4)); slip = np.zeros((n, 4)); twist = np.zeros(n)
    events, fell = [], None

    s = list(cfg.x0)
    cmd = (0.0, 0.0)
    req = np.zeros(4)
    pitch = roll = 0.0
    t0 = time.time()

    for k in range(n):
        t = t_arr[k]
        r = ref_fn(t)
        # --- kontrolcü (ZOH)
        if k % ctrl_every == 0:
            meas = s if k < dn or dn == 0 else ball[k - dn]
            rd = ref_fn(max(0.0, t - cfg.sensor_delay))   # hata, ölçümle aynı (gecikmiş) anda hesaplanır
            cmd = ctrl.compute(meas, (*rd[:4], r[4], r[5]))
            h = geom.inverse(cmd[0], cmd[1])
            h = np.clip(h, -geom.stroke, geom.stroke)          # strok sınırı
            req = geom.height_to_angle(h)
        # --- log (adım başındaki durum)
        ball[k] = s; ref[k] = r; tilt_cmd[k] = cmd; th_req[k] = req
        tilt[k] = (pitch, roll)
        for i, m in enumerate(motors):
            phi[k, i], th[k, i], slip[k, i] = m.phi, m.theta, m.slip
            po[k, i] = m.pullout_limit()
        # --- aktüatörler
        for i, m in enumerate(motors):
            if m.step(req[i], cfg.dt):
                events.append((t, i))
        theta = np.array([m.theta for m in motors])
        z0, pitch, roll, tw = geom.forward(geom.angle_to_height(theta))
        twist[k] = tw
        # --- top
        s = rk4_step(s, pitch, roll, cfg.dt, cfg.ball)
        if abs(s[0]) > geom.plate_half or abs(s[2]) > geom.plate_half:
            fell = t
            n_valid = k + 1
            break
    else:
        n_valid = n

    if verbose:
        print(f"simülasyon {time.time() - t0:.1f} s sürdü, senkron kaybı: {len(events)}"
              + (f", TOP DÜŞTÜ t={fell:.2f}s" if fell else ""))
    sl = slice(0, n_valid)
    return SimResult(cfg, t_arr[sl], ball[sl], ref[sl], tilt_cmd[sl], tilt[sl], th_req[sl],
                     phi[sl], th[sl], po[sl], slip[sl], twist[sl], events, fell)


def summarize(res: SimResult, label: str):
    e = res.ball[:, [0, 2]] - res.ref[:, [0, 2]]
    err = np.hypot(e[:, 0], e[:, 1])
    tail = res.t > res.t[-1] * 0.5
    print(f"[{label}] konum hatası: son yarı RMS={np.sqrt(np.mean(err[tail]**2))*1e3:.2f} mm, "
          f"max={err[tail].max()*1e3:.2f} mm; max eğim={np.degrees(np.abs(res.tilt).max()):.2f}°; "
          f"max |lead|={np.degrees(np.abs(res.phi-res.theta).max()):.2f}°")


def make_config(mode: str, stress: bool) -> tuple[SimConfig, object]:
    cfg = SimConfig()
    if mode == "balance":
        cfg.t_end = 12.0
        return cfg, StaticRef(0.0, 0.0)
    if stress:
        # zayıf motor (düşük pull-out) + hızlı yörünge: senkron kaybı görülür
        cfg.x0 = (0.0, 0.0, 0.0, 0.0)
        cfg.motor = StepperParams(pullout_full_steps=6.0)
        return cfg, LissajousRef(0.07, 0.05, period=6.0)
    return cfg, LissajousRef(0.07, 0.05, period=8.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["balance", "lissajous"], default="balance", nargs="?")
    ap.add_argument("--stress", action="store_true")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--out", default="output")
    ap.add_argument("--delay", type=float, default=None, help="ölçüm gecikmesi [s] (varsayılan 0.15; ~0.17 üstünde kararsız)")
    ap.add_argument("--no-anim", action="store_true")
    ap.add_argument("--exaggerate", type=float, default=10.0, help="3D animasyonda eğim büyütme")
    a = ap.parse_args()

    import matplotlib
    if not a.show:
        matplotlib.use("Agg")
    cfg, ref = make_config(a.mode, a.stress)
    if a.delay is not None:
        cfg.sensor_delay = a.delay
    res = run_simulation(cfg, ref)
    label = a.mode + ("_stress" if a.stress else "")
    summarize(res, label)

    import visualize as viz
    viz.make_all(res, a.out, label, show=a.show, animate=not a.no_anim, exaggerate=a.exaggerate)


if __name__ == "__main__":
    main()
