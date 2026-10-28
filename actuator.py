"""Step motor aktüatör modeli (birinci derece gecikme + doyum + pull-out).

Her motor iki katmandan oluşur:

1) Sürücü (açık çevrim step üreteci): istenen açıyı mikroadıma yuvarlar; adım
   frekansını f_max ile, adım frekansının değişimini (rampa) a_max ile sınırlar
   -> "komut açısı" (alan/stator konumu) phi. Rampa, gerçek sürücülerdeki
   trapez profil / start-stop frekansı sınırının karşılığıdır.

2) Rotor: komuta birinci derece gecikmeyle uyar,
        omega = clip( (phi - s - theta) / tau , ±omega_max )
   Burada lead = phi - s - theta yük açısıdır. Sürekli hızda lead ≈ omega*tau.
   Pull-out (senkron kaybı): |lead| > theta_po(omega) olursa rotor alanı kaçırır;
   rotor bir elektriksel periyot (4 tam adım) geride bir dengeye kayar, yani
   s += ±cycle ve bu fark açık çevrimde KALICI olur (kayıp adım).
   Pull-out açısı hızla azalır (tork-hız eğrisi):
        theta_po(omega) = theta_po0 * (1 - derate * |omega| / omega_max)

Parametreler tek-bir motor için toplanmış (lumped) değerlerdir. Gerçek motorda
tau, theta_po0, derate değerlerini motorun pull-out tork eğrisi ve yük eylemsizliğinden
ya da step-yanıtı ölçümünden uydur.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class StepperParams:
    full_steps_per_rev: int = 200       # 1.8°
    microstep: int = 16                 # mikroadım bölme
    max_step_rate: float = 800.0       # tam adım/s (f_max)
    max_step_accel: float = 30000.0     # tam adım/s^2 (sürücü rampası)
    tau: float = 0.008                  # s, birinci derece gecikme
    pullout_full_steps: float = 12.0    # theta_po0 (tam adım cinsinden yük açısı)
    derate: float = 0.4                 # hızla pull-out açısı düşüşü (0..1)

    @property
    def step_angle(self) -> float:
        return 2 * math.pi / self.full_steps_per_rev

    @property
    def microstep_angle(self) -> float:
        return self.step_angle / self.microstep

    @property
    def omega_max(self) -> float:
        return self.max_step_rate * self.step_angle        # rad/s

    @property
    def alpha_max(self) -> float:
        return self.max_step_accel * self.step_angle        # rad/s^2

    @property
    def theta_po0(self) -> float:
        return self.pullout_full_steps * self.step_angle

    @property
    def cycle(self) -> float:
        return 4 * self.step_angle                          # 1 elektriksel periyot


class StepperMotor:
    def __init__(self, params: StepperParams | None = None):
        self.p = params or StepperParams()
        self.theta = 0.0      # gerçek rotor açısı [rad]
        self.phi = 0.0        # sürücünün ürettiği komut (alan) açısı [rad]
        self.slip = 0.0       # kayıp adım ofseti s [rad]
        self.omega = 0.0
        self.v_drv = 0.0      # sürücü komut hızı [rad/s]
        self.lost_events = 0

    # --------------------------------------------------------------------
    def pullout_limit(self, omega: float | None = None) -> float:
        w = abs(self.omega if omega is None else omega)
        return self.p.theta_po0 * max(0.1, 1.0 - self.p.derate * w / self.p.omega_max)

    @property
    def lead(self) -> float:
        """Komut - rotor farı (lead>0: rotor geride)."""
        return self.phi - self.theta

    def step(self, theta_request: float, dt: float) -> bool:
        """dt kadar ilerlet. Senkron kaybı olduysa True döner."""
        p = self.p
        # --- sürücü: mikroadıma yuvarla + adım frekansı sınırı
        target = round(theta_request / p.microstep_angle) * p.microstep_angle
        err = target - self.phi
        # hız profili: f_max ve ivme sınırı (fren mesafesine göre)
        v_des = math.copysign(
            min(p.omega_max, math.sqrt(2.0 * p.alpha_max * abs(err)), abs(err) / dt), err
        )
        dv = p.alpha_max * dt
        self.v_drv += max(-dv, min(dv, v_des - self.v_drv))
        self.phi += self.v_drv * dt

        # --- rotor: birinci derece gecikme + hız doyumu
        lead_eff = self.phi - self.slip - self.theta
        w = lead_eff / p.tau
        w = max(-p.omega_max, min(p.omega_max, w))
        self.omega = w
        self.theta += w * dt

        # --- pull-out kontrolü
        lost = False
        lead_eff = self.phi - self.slip - self.theta
        if abs(lead_eff) > self.pullout_limit(w):
            # rotor, alanın en yakın kararlı denge noktasına (cycle katı) oturur
            n_cyc = max(1, round(abs(lead_eff) / p.cycle))
            self.slip += math.copysign(n_cyc * p.cycle, lead_eff)
            self.lost_events += 1
            lost = True
        return lost
