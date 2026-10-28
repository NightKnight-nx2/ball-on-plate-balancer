"""Dört bacaklı plaka için ters / ileri kinematik.

Düzen: dört dikey bacak, dünya (X, Y) düzleminde sabit noktalarda (dikdörtgen veya
herhangi bir dörtgen). Plaka merkezi (0, 0, z0), yalnızca pitch/roll ile eğilir.
Her bacak ucu plaka düzlemine temas eder.

Ters kinematik (tam, küçük açı varsayımı olmadan):
    n = (sin p, -sin r cos p, cos r cos p)            plaka normali
    z_i = z0 - (n_x X_i + n_y Y_i) / n_z
        = z0 - X_i tan(p)/cos(r) + Y_i tan(r)

Bacak yüksekliği -> step motor: vidalı mil (lead screw) ile  z = lead * theta / (2 pi).

NOT: Plaka rijit ve 4 bacak 3 serbestlik derecesini (z0, pitch, roll) aşırı belirler.
Komutlar düzlemsel (tutarlı) olduğu için sorun yok; ama motorlar farklı hata yaparsa
(gecikme, senkron kaybı) 4 bacak aynı düzlemde olmaz. İleri kinematik bu durumda
en küçük kareler düzlemi uydurur ve kalıntıyı ("twist") raporlar – gerçekte bu fark
mafsal/plaka esnekliği ile karşılanır.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def _default_legs(half_x: float = 0.15, half_y: float = 0.15) -> np.ndarray:
    # Sıra: 0:(+,+) 1:(-,+) 2:(-,-) 3:(+,-)
    return np.array(
        [[half_x, half_y], [-half_x, half_y], [-half_x, -half_y], [half_x, -half_y]]
    )


@dataclass
class PlateGeometry:
    leg_xy: np.ndarray = field(default_factory=_default_legs)  # (4, 2) m
    plate_half: float = 0.15          # m, plaka yarı kenar (top düşme sınırı)
    lead: float = 0.008               # m/tur, vidalı mil adımı (T8x8)
    stroke: float = 0.015             # m, bacak ±strok sınırı
    z0: float = 0.0                   # m, plaka merkezi nominal yüksekliği

    def __post_init__(self):
        self.leg_xy = np.asarray(self.leg_xy, dtype=float)
        X, Y = self.leg_xy[:, 0], self.leg_xy[:, 1]
        # İleri kinematik için en küçük kareler matrisi: z_i = c0 + c1 X_i + c2 Y_i
        M = np.column_stack([np.ones(4), X, Y])
        self._pinv = np.linalg.pinv(M)
        self._M = M

    # ---- tilt <-> bacak yükseklikleri -------------------------------------
    def inverse(self, pitch: float, roll: float, z0: float | None = None) -> np.ndarray:
        """(pitch, roll) -> 4 bacak yüksekliği [m]. Strok sınırına kırpılmaz."""
        z0 = self.z0 if z0 is None else z0
        X, Y = self.leg_xy[:, 0], self.leg_xy[:, 1]
        return z0 - X * np.tan(pitch) / np.cos(roll) + Y * np.tan(roll)

    def forward(self, heights: np.ndarray):
        """4 bacak yüksekliği -> (z0, pitch, roll, twist).

        twist: en küçük kareler düzlemine göre kalıntının RMS'i [m]."""
        c = self._pinv @ np.asarray(heights, dtype=float)
        c0, c1, c2 = c
        n = np.array([-c1, -c2, 1.0])
        n /= np.linalg.norm(n)
        pitch = np.arctan2(n[0], np.hypot(n[1], n[2]))
        roll = np.arctan2(-n[1], n[2])
        resid = self._M @ c - heights
        return c0, pitch, roll, float(np.sqrt(np.mean(resid**2)))

    # ---- bacak <-> motor ---------------------------------------------------
    def height_to_angle(self, h):
        return 2.0 * np.pi * np.asarray(h) / self.lead

    def angle_to_height(self, theta):
        return np.asarray(theta) * self.lead / (2.0 * np.pi)

    def max_tilt(self) -> float:
        """Strok sınırını aşmayan yaklaşık maksimum eğim (rad)."""
        reach = np.max(np.abs(self.leg_xy))
        return float(np.arcsin(self.stroke / (np.sqrt(2) * reach)))

    # ---- 3D çizim yardımcıları ---------------------------------------------
    @staticmethod
    def rotation(pitch: float, roll: float) -> np.ndarray:
        cp, sp_ = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        Ry = np.array([[cp, 0, sp_], [0, 1, 0], [-sp_, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rx @ Ry


if __name__ == "__main__":
    g = PlateGeometry()
    p, r = 0.03, -0.02
    h = g.inverse(p, r)
    print("heights [mm]:", h * 1e3)
    z0, p2, r2, tw = g.forward(h)
    print("roundtrip:", z0, p2, r2, tw)
    print("max tilt [deg]:", np.degrees(g.max_tilt()))
