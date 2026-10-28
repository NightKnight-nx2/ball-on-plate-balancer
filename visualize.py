"""Grafikler ve 3D animasyon (matplotlib).

(a) plot_trajectory : topun 2D yörüngesi + referans
(b) plot_phase      : konum/hız hatası faz portresi (x ve y ekseni)
(c) plot_actuators  : 4 step motor - komut vs gerçek açı, lead, pull-out sınırları
(d) animate_3d      : plaka + 4 bacak + top animasyonu (mplot3d)
"""
from __future__ import annotations

import os
import shutil

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

# Okabe-Ito (renk körlüğüne dayanıklı) – sabit sırayla
C_BALL, C_REF, C_CMD, C_LIM, C_BAD = "#0072B2", "#D55E00", "#009E73", "#444444", "#CC79A7"
MOTOR_COLORS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]

plt.rcParams.update({
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": 0.25, "grid.linewidth": 0.6,
    "lines.linewidth": 1.6, "font.size": 9,
})


# ---------------------------------------------------------------- (a)
def plot_trajectory(res, path=None):
    fig, ax = plt.subplots(figsize=(6.2, 6))
    h = res.cfg.geom.plate_half
    ax.add_patch(plt.Rectangle((-h, -h), 2 * h, 2 * h, fill=False, ec="#999", lw=1))
    ax.plot(res.ref[:, 0] * 1e3, res.ref[:, 2] * 1e3, "--", c=C_REF, lw=1.4, label="referans")
    ax.plot(res.ball[:, 0] * 1e3, res.ball[:, 2] * 1e3, c=C_BALL, label="top")
    ax.plot(*(res.ball[0, [0, 2]] * 1e3), "o", c=C_BALL, ms=7, label="başlangıç")
    ax.plot(*(res.ball[-1, [0, 2]] * 1e3), "s", c=C_BALL, ms=7, label="bitiş")
    ax.set_xlim(-h * 1e3 * 1.05, h * 1e3 * 1.05); ax.set_ylim(-h * 1e3 * 1.05, h * 1e3 * 1.05)
    ax.set_aspect("equal"); ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]")
    ax.set_title("Topun plaka üzerindeki 2D yörüngesi (plaka çerçevesi gri)")
    ax.legend(loc="upper right", frameon=False)
    return _finish(fig, path)


# ---------------------------------------------------------------- (b)
def plot_phase(res, path=None):
    e = res.ball[:, [0, 2]] - res.ref[:, [0, 2]]
    ed = res.ball[:, [1, 3]] - res.ref[:, [1, 3]]
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.8))
    for i, (ax, nm) in enumerate(zip(axs, "xy")):
        px, py = e[:, i] * 1e3, ed[:, i] * 1e3
        pts = np.column_stack([px, py]).reshape(-1, 1, 2)
        lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], axis=1), cmap="viridis",
                            norm=plt.Normalize(res.t[0], res.t[-1]), linewidth=1.6)
        lc.set_array(res.t[:-1]); ax.add_collection(lc)
        ax.plot(px[0], py[0], "o", c="k", ms=7, label="başlangıç")
        ax.plot(0, 0, "+", c=C_REF, ms=14, mew=2, label="denge")
        ax.set_xlim(px.min() * 1.1 - 1, px.max() * 1.1 + 1); ax.set_ylim(py.min() * 1.1 - 1, py.max() * 1.1 + 1)
        ax.set_xlabel(f"{nm} konum hatası [mm]"); ax.set_ylabel(f"{nm} hız hatası [mm/s]")
        ax.set_title(f"Faz portresi ({nm} ekseni)"); ax.legend(frameon=False, loc="best")
        if i == 1:
            fig.colorbar(lc, ax=axs, label="zaman [s]", fraction=0.025, pad=0.02)
    return _finish(fig, path)


# ---------------------------------------------------------------- (c)
def plot_actuators(res, path=None):
    t = res.t
    fig, axs = plt.subplots(4, 2, figsize=(12, 9), sharex=True)
    deg = np.degrees
    for i in range(4):
        c = MOTOR_COLORS[i]
        tgt = res.phi[:, i] - res.slip[:, i]           # rotorun izlediği alan konumu
        ax = axs[i, 0]
        ax.plot(t, deg(res.theta_req[:, i]), c="#bbb", lw=0.8, label="istek (ham)")
        ax.plot(t, deg(res.phi[:, i]), c=C_CMD, lw=1.4, label="komut (sürücü)")
        ax.plot(t, deg(res.theta[:, i]), c=c, lw=1.4, label="gerçek rotor")
        ax.plot(t, deg(tgt + res.po_limit[:, i]), "--", c=C_LIM, lw=0.9, label="pull-out sınırı")
        ax.plot(t, deg(tgt - res.po_limit[:, i]), "--", c=C_LIM, lw=0.9)
        ax.set_ylabel(f"Motor {i+1} [°]")
        ax = axs[i, 1]
        lead = res.phi[:, i] - res.slip[:, i] - res.theta[:, i]
        ax.plot(t, deg(lead), c=c, label="lead = komut − rotor")
        ax.plot(t, deg(res.po_limit[:, i]), "--", c=C_LIM, lw=0.9, label="±pull-out")
        ax.plot(t, -deg(res.po_limit[:, i]), "--", c=C_LIM, lw=0.9)
        if np.any(res.slip[:, i] != 0):
            ax.plot(t, deg(res.slip[:, i]), ":", c=C_BAD, lw=1.4, label="kayıp adım ofseti")
        ev = [te for te, m in res.sync_events if m == i]
        if ev:
            ax.plot(ev, np.zeros(len(ev)), "x", c="#CC0000", ms=6, label=f"senkron kaybı ({len(ev)})")
        ax.set_ylabel("lead [°]")
        if i == 0:
            axs[i, 0].legend(frameon=False, fontsize=7, ncol=4, loc="lower right", bbox_to_anchor=(1, 1.0))
            axs[i, 1].legend(frameon=False, fontsize=7, ncol=4, loc="lower right", bbox_to_anchor=(1, 1.0))
    axs[0, 0].set_title("Komut edilen vs gerçek rotor açısı", loc="left", pad=22)
    axs[0, 1].set_title("Yük açısı (lead/lag)", loc="left", pad=22)
    axs[-1, 0].set_xlabel("zaman [s]"); axs[-1, 1].set_xlabel("zaman [s]")
    fig.tight_layout()
    return _finish(fig, path)


# ---------------------------------------------------------------- (d)
def animate_3d(res, path_gif, exaggerate=10.0, fps=20, speed=1.0):
    from matplotlib.animation import FuncAnimation, PillowWriter
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    geom, bp = res.cfg.geom, res.cfg.ball
    stride = max(1, int(round(speed / (fps * res.cfg.dt))))
    idx = np.arange(0, len(res.t), stride)
    h = geom.plate_half
    corners = np.array([[h, h, 0], [-h, h, 0], [-h, -h, 0], [h, -h, 0]]).T
    base_z = -0.06
    ref_pts = np.column_stack([res.ref[::20, 0], res.ref[::20, 2], np.full(len(res.ref[::20]), bp.radius * 0)]).T

    fig = plt.figure(figsize=(6.4, 5.4))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-0.2, 0.2); ax.set_ylim(-0.2, 0.2); ax.set_zlim(base_z, 0.09)
    ax.set_box_aspect((1, 1, 0.55)); ax.view_init(elev=22, azim=-58)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    plate = Poly3DCollection([np.zeros((4, 3))], alpha=0.35, fc="#8fb4d9", ec="#235", lw=1.2)
    ax.add_collection3d(plate)
    legs = [ax.plot([], [], [], c=MOTOR_COLORS[i], lw=4, solid_capstyle="round")[0] for i in range(4)]
    for i, (X, Y) in enumerate(geom.leg_xy):
        ax.text(X * 1.25, Y * 1.25, base_z, f"M{i+1}", color=MOTOR_COLORS[i], ha="center")
    refline, = ax.plot([], [], [], "--", c=C_REF, lw=1)
    trail, = ax.plot([], [], [], c=C_BALL, lw=1.2, alpha=0.8)
    ball, = ax.plot([], [], [], "o", c=C_BALL, ms=11, mec="k")
    txt = ax.set_title("")
    trail_n = int(1.5 / (stride * res.cfg.dt))
    trail_buf = []

    def update(f):
        k = idx[f]
        p, r = res.tilt[k] * exaggerate
        R = geom.rotation(p, r)
        z0 = geom.z0
        pc = (R @ corners).T + [0, 0, z0]
        plate.set_verts([pc])
        zl = geom.inverse(p, r)
        for i, (X, Y) in enumerate(geom.leg_xy):
            legs[i].set_data_3d([X, X], [Y, Y], [base_z, zl[i]])
        rp = (R @ ref_pts).T
        refline.set_data_3d(rp[:, 0], rp[:, 1], rp[:, 2] + z0)
        bw = R @ np.array([res.ball[k, 0], res.ball[k, 2], bp.radius]) + [0, 0, z0]
        trail_buf.append(R @ np.array([res.ball[k, 0], res.ball[k, 2], 0.0]) + [0, 0, z0])
        if len(trail_buf) > trail_n:
            trail_buf.pop(0)
        tb = np.array(trail_buf)
        trail.set_data_3d(tb[:, 0], tb[:, 1], tb[:, 2])
        ball.set_data_3d([bw[0]], [bw[1]], [bw[2]])
        txt.set_text(f"t = {res.t[k]:5.2f} s   (eğim ×{exaggerate:g} abartılı)")
        return []

    anim = FuncAnimation(fig, update, frames=len(idx), interval=1000 / fps, blit=False)
    if path_gif:
        anim.save(path_gif, writer=PillowWriter(fps=fps), dpi=70)
        print("kaydedildi:", path_gif)
        if shutil.which("ffmpeg"):
            mp4 = os.path.splitext(path_gif)[0] + ".mp4"
            trail_buf.clear()
            anim.save(mp4, writer="ffmpeg", fps=fps, dpi=90)
            print("kaydedildi:", mp4)
    return anim


# ---------------------------------------------------------------- yardımcılar
def _finish(fig, path):
    if path:
        fig.savefig(path, dpi=130, bbox_inches="tight")
        print("kaydedildi:", path)
    return fig


def make_all(res, outdir, label, show=False, animate=True, exaggerate=10.0):
    os.makedirs(outdir, exist_ok=True)
    f = lambda n: os.path.join(outdir, f"{label}_{n}")
    plot_trajectory(res, f("a_yorunge.png"))
    plot_phase(res, f("b_faz_portresi.png"))
    plot_actuators(res, f("c_motorlar.png"))
    anim = None
    if animate:
        anim = animate_3d(res, f("d_animasyon.gif"), exaggerate=exaggerate)
    if show:
        plt.show()
    return anim
