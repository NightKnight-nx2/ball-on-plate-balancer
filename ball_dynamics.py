"""Top-plaka (ball-on-plate) dinamiği.

Model: plaka üzerinde kaymadan yuvarlanan homojen küre (kütle noktası + dönme
eylemsizliği). Plaka sabit bir eksen takımına göre yalnızca eğiliyor kabul edilir
(plaka açısal hızından gelen merkezkaç/Coriolis terimleri ihmal edilir – literatürdeki
standart basitleştirme).

Açı tanımı (plaka dönüşü R = Rx(roll) @ Ry(pitch)):
    pitch : y ekseni etrafında dönüş  -> +pitch, +x yönünde ivme verir
    roll  : x ekseni etrafında dönüş  -> +roll,  -y yönünde ivme verir
Plaka normali n = (sin p, -sin r cos p, cos r cos p).

Türetim sympy ile Euler-Lagrange üzerinden yapılır:
    L = 1/2 (m + I/r_b^2) (xd^2 + yd^2) - m g z(x, y)
    z = -x sin p cos r + y sin r        (topun plaka koordinatlarında yüksekliği)
    => (m + I/r_b^2) xdd = m g sin p cos r
       (m + I/r_b^2) ydd = -m g sin r
Rolling-without-slipping kısıtı, kinetik enerjideki I/r_b^2 terimi ile zaten
gömülüdür (omega = v / r_b).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np
import sympy as sp


@dataclass
class BallParams:
    mass: float = 0.030          # kg
    radius: float = 0.020        # m
    beta: float = 0.4            # I = beta*m*r^2  (dolu küre 2/5, içi boş küre 2/3)
    g: float = 9.81              # m/s^2
    damping: float = 0.05        # 1/s, viskoz yuvarlanma sönümü (xdd -= c*xd)

    @property
    def k(self) -> float:
        """Kazanç: küçük açıda ivme = k * açı  (dolu küre için 5g/7)."""
        return self.g / (1.0 + self.beta)


@lru_cache(maxsize=1)
def derive_symbolic():
    """Sympy ile hareket denklemlerini türetir. Sonuçlar semboliktir (önbelleğe alınır)."""
    t = sp.symbols("t")
    m, rb, g, beta, c = sp.symbols("m r_b g beta c", positive=True)
    p, r = sp.symbols("pitch roll")                     # plaka açıları (girdi)
    x, y = sp.Function("x")(t), sp.Function("y")(t)

    inertia = beta * m * rb**2
    z = -x * sp.sin(p) * sp.cos(r) + y * sp.sin(r)
    T = sp.Rational(1, 2) * (m + inertia / rb**2) * (x.diff(t) ** 2 + y.diff(t) ** 2)
    V = m * g * z
    L = T - V

    eqs = []
    for q in (x, y):
        el = sp.diff(L.diff(q.diff(t)), t) - L.diff(q)
        eqs.append(sp.simplify(el))
    xdd, ydd = sp.symbols("xdd ydd")
    sol = sp.solve(
        [e.subs({x.diff(t, 2): xdd, y.diff(t, 2): ydd}) for e in eqs], [xdd, ydd]
    )
    xdd_expr = sp.simplify(sol[xdd])
    ydd_expr = sp.simplify(sol[ydd])

    # Viskoz sönüm (Rayleigh): ivmeden -c*v çıkarılır
    vx, vy = sp.symbols("vx vy")
    xdd_expr = xdd_expr - c * vx
    ydd_expr = ydd_expr - c * vy

    X = sp.symbols("x")  # konum sembolü (dinamik konuma bağlı değil, Jacobian için)
    Y = sp.symbols("y")
    state = sp.Matrix([X, vx, Y, vy])
    f = sp.Matrix([vx, xdd_expr, vy, ydd_expr])
    u = sp.Matrix([p, r])

    A_full = f.jacobian(state)
    B_full = f.jacobian(u)
    # Denge: p = r = 0, v = 0
    eq = {p: 0, r: 0, vx: 0, vy: 0}
    A_lin = sp.simplify(A_full.subs(eq))
    B_lin = sp.simplify(B_full.subs(eq))

    return dict(
        symbols=dict(m=m, rb=rb, g=g, beta=beta, c=c, p=p, r=r, vx=vx, vy=vy),
        xdd=xdd_expr, ydd=ydd_expr, f=f, state=state, u=u, A=A_lin, B=B_lin,
    )


@lru_cache(maxsize=1)
def _lambdified():
    d = derive_symbolic()
    s = d["symbols"]
    args = (s["p"], s["r"], s["vx"], s["vy"], s["m"], s["rb"], s["g"], s["beta"], s["c"])
    ax = sp.lambdify(args, d["xdd"], modules="math")
    ay = sp.lambdify(args, d["ydd"], modules="math")
    A = sp.lambdify((s["m"], s["rb"], s["g"], s["beta"], s["c"]), d["A"], modules="numpy")
    B = sp.lambdify((s["m"], s["rb"], s["g"], s["beta"], s["c"]), d["B"], modules="numpy")
    return ax, ay, A, B


def ball_accel(pitch: float, roll: float, vx: float, vy: float, bp: BallParams):
    """Doğrusal olmayan top ivmesi (plaka koordinatlarında)."""
    ax, ay, _, _ = _lambdified()
    a = (pitch, roll, vx, vy, bp.mass, bp.radius, bp.g, bp.beta, bp.damping)
    return ax(*a), ay(*a)


def ball_deriv(state, pitch: float, roll: float, bp: BallParams):
    """state = [x, vx, y, vy] -> türev."""
    x, vx, y, vy = state
    ax, ay = ball_accel(pitch, roll, vx, vy, bp)
    return (vx, ax, vy, ay)


def rk4_step(state, pitch: float, roll: float, dt: float, bp: BallParams):
    """Plaka açıları adım boyunca sabit (ZOH) kabul edilerek RK4 ile ilerlet."""
    s = state
    k1 = ball_deriv(s, pitch, roll, bp)
    k2 = ball_deriv([s[i] + 0.5 * dt * k1[i] for i in range(4)], pitch, roll, bp)
    k3 = ball_deriv([s[i] + 0.5 * dt * k2[i] for i in range(4)], pitch, roll, bp)
    k4 = ball_deriv([s[i] + dt * k3[i] for i in range(4)], pitch, roll, bp)
    return [s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(4)]


def linearize(bp: BallParams):
    """Denge etrafında doğrusallaştırma: durum [x, vx, y, vy], girdi [pitch, roll]."""
    _, _, A, B = _lambdified()
    A = np.array(A(bp.mass, bp.radius, bp.g, bp.beta, bp.damping), dtype=float)
    B = np.array(B(bp.mass, bp.radius, bp.g, bp.beta, bp.damping), dtype=float)
    return A, B


if __name__ == "__main__":
    d = derive_symbolic()
    print("xdd =", d["xdd"])
    print("ydd =", d["ydd"])
    print("A =", d["A"])
    print("B =", d["B"])
    bp = BallParams()
    A, B = linearize(bp)
    print("k = g/(1+beta) =", bp.k)
    print(A, B, sep="\n")
