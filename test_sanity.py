"""Hızlı doğrulama:  python -m pytest -q   (veya  python test_sanity.py)"""
import numpy as np

from actuator import StepperMotor, StepperParams
from ball_dynamics import BallParams, ball_accel, linearize
from controller import BallPlateLQR
from kinematics import PlateGeometry
from simulate import make_config, run_simulation


def test_dynamics_matches_literature():
    bp = BallParams(beta=0.4, damping=0.0)
    ax, ay = ball_accel(0.01, 0.0, 0, 0, bp)
    assert abs(ax - 5 / 7 * 9.81 * np.sin(0.01)) < 1e-9          # (5/7) g sin(theta)
    A, B = linearize(bp)
    assert abs(B[1, 0] - 5 / 7 * 9.81) < 1e-9 and abs(B[3, 1] + 5 / 7 * 9.81) < 1e-9


def test_kinematics_roundtrip():
    g = PlateGeometry()
    for p, r in [(0.03, -0.02), (-0.05, 0.04), (0.0, 0.0)]:
        z0, p2, r2, tw = g.forward(g.inverse(p, r))
        assert abs(p - p2) < 1e-12 and abs(r - r2) < 1e-12 and tw < 1e-12 and abs(z0) < 1e-12


def test_lqr_stable():
    c = BallPlateLQR(BallParams())
    assert np.all(np.linalg.eigvals(c.A - c.B @ c.K).real < 0)


def test_stepper_loses_sync_when_overdriven():
    m = StepperMotor(StepperParams(pullout_full_steps=1.0))
    lost = any(m.step(50.0, 1e-3) for _ in range(2000))
    assert lost and m.slip != 0


def test_balance_converges():
    cfg, ref = make_config("balance", False)
    cfg.t_end = 8.0
    res = run_simulation(cfg, ref, verbose=False)
    assert res.fell_off is None and np.hypot(*res.ball[-1, [0, 2]]) < 1e-3
    assert len(res.sync_events) == 0


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f(); print("ok", n)
