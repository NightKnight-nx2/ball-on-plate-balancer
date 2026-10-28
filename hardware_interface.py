"""Gerçek donanıma (Arduino + step motor sürücü + kamera) geçiş iskeleti.

ŞİMDİLİK BOŞ/MOCK: simülasyon kodundaki aynı denetleyici ve kinematik, bu arayüz
üzerinden gerçek donanıma bağlanabilsin diye soyutlama bırakıldı.

Önerilen akış (bkz. run_control_loop):
    kamera -> top durumu [x, vx, y, vy] -> BallPlateLQR.compute -> PlateGeometry.inverse
    -> bacak yüksekliği -> motor açısı -> adım sayısı -> Arduino

Yapılacaklar (donanım aşamasında):
  * SerialHardware: pyserial ile Arduino protokolü (ör. "M<i> <hedef_adım>\\n"), AccelStepper
    veya benzeri ile ivme rampası; hız/ivme sınırlarını actuator.StepperParams ile eşle.
  * Kamera ölçümü: konum + hız tahmini (Kalman/alpha-beta), ölçüm gecikmesi simülasyondaki
    cfg.sensor_delay ile uyumlu olmalı.
  * Homing: plakayı seviyeye getir, adım sayaçlarını sıfırla (açık çevrim -> senkron kaybı
    algılanamaz; gerekirse limit anahtarı/enkoder ekle).
"""
from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod

import numpy as np

from actuator import StepperMotor, StepperParams
from kinematics import PlateGeometry


class HardwareInterface(ABC):
    n_motors = 4

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def home(self) -> None:
        """Bacakları referans (seviye) konuma getir, adım sayaçlarını sıfırla."""

    @abstractmethod
    def read_ball_state(self) -> tuple[float, float, float, float]:
        """[x, vx, y, vy] (m, m/s) – kamera/estimator çıktısı, plaka çerçevesinde."""

    @abstractmethod
    def send_motor_angles(self, angles_rad: np.ndarray) -> None:
        """4 motor için hedef mutlak açı [rad] (sürücü tarafı adıma çevirir)."""

    @abstractmethod
    def close(self) -> None: ...


class MockHardware(HardwareInterface):
    """Donanım yokken arayüzü denemek için: motorlar actuator.StepperMotor, top sabit."""

    def __init__(self, params: StepperParams | None = None):
        self.motors = [StepperMotor(params) for _ in range(4)]
        self.ball = (0.0, 0.0, 0.0, 0.0)
        self._t = None

    def connect(self): self._t = time.time()
    def home(self): [setattr(m, a, 0.0) for m in self.motors for a in ("theta", "phi", "slip", "omega", "v_drv")]
    def read_ball_state(self): return self.ball

    def send_motor_angles(self, angles_rad):
        now = time.time(); dt = min(now - self._t, 0.05); self._t = now
        for m, a in zip(self.motors, angles_rad):
            m.step(float(a), dt)

    def close(self): pass


class SerialHardware(HardwareInterface):
    """Arduino + kamera için iskelet (uygulanmadı)."""

    def __init__(self, port: str = "COM3", baud: int = 115200, steps_per_rad: float | None = None):
        self.port, self.baud = port, baud
        self.steps_per_rad = steps_per_rad or (200 * 16 / (2 * math.pi))

    def connect(self): raise NotImplementedError("pyserial ile bağlantı eklenecek")
    def home(self): raise NotImplementedError
    def read_ball_state(self): raise NotImplementedError("kamera + estimator eklenecek")
    def send_motor_angles(self, angles_rad): raise NotImplementedError
    def close(self): raise NotImplementedError


def run_control_loop(hw: HardwareInterface, controller, geom: PlateGeometry, ref_fn,
                     rate_hz: float = 100.0, duration: float = 10.0):
    """Gerçek zamanlı döngü örneği (simülasyondaki ile aynı katmanlar)."""
    hw.connect(); hw.home()
    t0 = time.time(); period = 1.0 / rate_hz
    try:
        while (t := time.time() - t0) < duration:
            s = hw.read_ball_state()
            pitch, roll = controller.compute(s, ref_fn(t))
            h = np.clip(geom.inverse(pitch, roll), -geom.stroke, geom.stroke)
            hw.send_motor_angles(geom.height_to_angle(h))
            time.sleep(max(0.0, period - ((time.time() - t0) - t)))
    finally:
        hw.close()
