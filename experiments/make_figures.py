"""Make every figure and table used in the report and slides.

    python experiments/make_figures.py

Inputs : results/experiments.csv (from run_experiments.py) + fresh,
         seeded demo missions (re-simulated here, deterministic).
Outputs: results/figures/*.png, results/tables/*.tex, results/summary.md
"""
import csv
import math
import os
import statistics as st
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                     # noqa: E402
import matplotlib.patches as mp                     # noqa: E402
from scipy import signal                            # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from valet.config import DEFAULT                                 # noqa: E402
from valet.costmap import CostMap                                # noqa: E402
from valet.geometry import car_footprint                         # noqa: E402
from valet.lot import make_scenario                              # noqa: E402
from valet.planning import HybridAStarPlanner, GridAStarPlanner  # noqa: E402
from sim.simulator import Simulator                              # noqa: E402

FIG = os.path.join(ROOT, "results", "figures")
TAB = os.path.join(ROOT, "results", "tables")
os.makedirs(FIG, exist_ok=True)
os.makedirs(TAB, exist_ok=True)

# Validated categorical palette (dataviz reference, slots 1-3), text in ink tones.
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, CARGREY = "#0b0b0b", "#52514e", "#e4e3df", "#b9b8b2"
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2,
    "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
    "lines.linewidth": 2.0, "savefig.dpi": 200, "savefig.bbox": "tight"})
P = DEFAULT


def save(fig, name):
    fig.savefig(os.path.join(FIG, name))
    plt.close(fig)
    print("  figure", name)


# ----------------------------------------------------------------- drawing
def draw_lot(ax, sc, labels=True, highlight=None):
    lp = P.lot
    ax.add_patch(mp.Rectangle((0, 0), lp.width, lp.height, fc="#f4f3f0", ec="none", zorder=0))
    for s in sc.slots:
        r = s.rect()
        fc = "#dff0e6" if (highlight and s.sid == highlight) else "none"
        ax.add_patch(mp.Polygon(r.corners(), fill=fc != "none", fc=fc, ec="#9a9994", lw=0.5, zorder=1))
        if labels:
            ax.text(s.cx, s.cy + s.opening * 2.35, s.sid, ha="center", va="center", fontsize=5.5,
                    color=INK, zorder=3, bbox=dict(boxstyle="square,pad=0.1", fc="white", ec="none", alpha=0.8))
    for b in sc.walls:
        ax.add_patch(mp.Polygon(b.corners(), fc="#6b6a66", ec="none", zorder=2))
    for b in sc.parked:
        ax.add_patch(mp.Polygon(b.corners(), fc=CARGREY, ec="#8d8c87", lw=0.5, zorder=2))
    ax.set_aspect("equal")
    ax.set_xlim(-1, lp.width + 1)
    ax.set_ylim(-1, lp.height + 1)
    ax.grid(False)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def draw_car(ax, x, y, yaw, color, lw=0.8, fill=False, alpha=1.0, z=4):
    fp = car_footprint(x, y, yaw, P.vehicle)
    ax.add_patch(mp.Polygon(fp.corners(), fill=fill, fc=color, ec=color, lw=lw, alpha=alpha, zorder=z))


# ------------------------------------------------------------- figures 1-2
def fig_lot_and_sensors():
    sc = make_scenario(seed=3, occupancy=0.7, n_pedestrians=2)
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    draw_lot(ax, sc)
    x, y, yaw = sc.start
    draw_car(ax, x, y, yaw, C1, fill=True, alpha=0.9)
    ax.annotate("drop-off /\nstart", (x, y + 1.5), (x + 2.2, y + 6), fontsize=8, color=INK,
                arrowprops=dict(arrowstyle="->", color=INK2, lw=0.8))
    ex, ey = sc.exit_point
    ax.add_patch(mp.Rectangle((ex - 2.5, ey - 2.0), 5, 4, fc="none", ec=C3, lw=1.5, ls="--", zorder=3))
    ax.text(ex, ey, "pick-up\nzone", ha="center", va="center", fontsize=8, color=INK)
    for ped in sc.pedestrians:
        ax.plot([ped.ax, ped.bx], [ped.ay, ped.by], color=C2, lw=1.0, ls=":")
        ax.add_patch(mp.Circle((ped.ax, ped.ay), 0.35, fc=C2, ec="white", lw=1, zorder=5))
    ax.text(20, 9, "aisle 1 (7 m)", ha="center", fontsize=8, color=INK2)
    ax.text(20, 27, "aisle 2 (7 m)", ha="center", fontsize=8, color=INK2)
    ax.set_title("Parking-lot world: 48 slots in 4 rows, 70 % occupied", fontsize=10, color=INK)
    save(fig, "fig_lot_layout.png")

    # sensor layout
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    draw_car(ax, 0, 0, 0, C1, lw=1.5, fill=True, alpha=0.15)
    vp = P.vehicle
    for xw in (0, vp.wheelbase):
        for yw in (-0.8, 0.8):
            ax.add_patch(mp.Rectangle((xw - 0.35, yw - 0.12), 0.7, 0.24, fc=INK2, zorder=5))
    for name, sx, sy, a in P.sensors.mounts:
        L = 1.6
        col = C2 if name.startswith(("front", "rear")) else C3
        ax.plot([sx, sx + L * math.cos(a)], [sy, sy + L * math.sin(a)], color=col, lw=1.0)
        ax.plot(sx, sy, "o", ms=3.5, color=col, zorder=6)
    ax.plot(0, 0, "+", ms=10, color=INK, mew=1.5, zorder=7)
    ax.annotate("reference point:\nrear-axle centre", (0, 0), (-3.3, 1.9), fontsize=8,
                arrowprops=dict(arrowstyle="->", color=INK2, lw=0.8))
    ax.plot([], [], color=C2, label="bumper sensors (7 front + 7 rear)")
    ax.plot([], [], color=C3, label="side sensors (6 per side)")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.42), ncol=2, fontsize=8)
    ax.set_aspect("equal"); ax.set_xlim(-3.5, 6.5); ax.set_ylim(-2.9, 2.9)
    ax.set_xlabel("x [m]  (car frame)"); ax.set_ylabel("y [m]")
    save(fig, "fig_sensor_layout.png")


# ---------------------------------------------------------------- figure 3
def fig_planner_comparison():
    sc = make_scenario(seed=1, occupancy=0.7)
    cm = CostMap(sc.obstacles)
    slot = sc.slot("B3")
    hp = HybridAStarPlanner(cm, use_funnel=True, record_tree=True)
    hpath, hinfo = hp.plan(sc.start, slot.park_pose())
    hp2 = HybridAStarPlanner(cm, use_funnel=False, record_tree=True)
    _, ninfo = hp2.plan(sc.start, slot.park_pose())
    gpath, ginfo = GridAStarPlanner(cm).plan(sc.start, slot)

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.9))
    for ax in axes:
        draw_lot(ax, sc, labels=False, highlight="B3")
        ax.set_xlim(0, 26); ax.set_ylim(0, 20)
    ax = axes[0]
    ax.plot(gpath.x, gpath.y, color=C2, lw=1.8, ls="--", label="Grid A* (ignores heading)", zorder=5)
    ax.plot(hpath.x, hpath.y, color=C1, lw=1.8, label="Hybrid A* + funnel", zorder=6)
    for i in range(0, len(hpath), 7):
        draw_car(ax, hpath.x[i], hpath.y[i], hpath.yaw[i], C1, lw=0.5, alpha=0.6)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("(a) Planned routes to slot B3", fontsize=9.5)
    ax = axes[1]
    for (x0, y0, x1, y1) in hp2.tree[::2]:
        ax.plot([x0, x1], [y0, y1], color=C2, lw=0.3, alpha=0.5, zorder=3)
    for (x0, y0, x1, y1) in hp.tree:
        ax.plot([x0, x1], [y0, y1], color=C1, lw=0.4, alpha=0.8, zorder=4)
    ax.plot([], [], color=C2, label=f"no funnel: {ninfo['expansions']:,} expansions")
    ax.plot([], [], color=C1, label=f"goal funnel: {hinfo['expansions']:,} expansions")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title("(b) Hybrid A* search trees", fontsize=9.5)
    save(fig, "fig_planner_paths.png")
    return hinfo, ninfo, ginfo


# ---------------------------------------------------------- figures 4-6
def demo_runs():
    sc_kw = dict(seed=3, occupancy=0.7)
    runs = {}
    for law in ("pure_pursuit", "stanley"):
        s = Simulator(make_scenario(**sc_kw), seed=3, law=law)
        s.run()
        runs[law] = s
    return runs


def fig_mission(runs):
    s = runs["pure_pursuit"]
    log = s.log
    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    draw_lot(ax, s.sc, labels=False, highlight=s.agent.target.sid)
    ax.set_xlim(0, 28); ax.set_ylim(0, 20)
    p = s.agent.path
    ax.plot(p.x, p.y, color=INK2, lw=1.0, ls="--", label="planned path", zorder=5)
    xs, ys = [r["x"] for r in log], [r["y"] for r in log]
    ax.plot(xs, ys, color=C1, lw=1.8, label="driven (rear axle)", zorder=6)
    for k in range(0, len(log), 40):
        r = log[k]
        col = C1 if r["v"] >= 0 else C2
        draw_car(ax, r["x"], r["y"], r["yaw"], col, lw=0.6, alpha=0.8)
    r = log[-1]
    draw_car(ax, r["x"], r["y"], r["yaw"], C3, lw=1.2, fill=True, alpha=0.5, z=7)
    ax.plot([], [], color=C1, lw=0.8, label="footprint, forward")
    ax.plot([], [], color=C2, lw=0.8, label="footprint, reverse")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.set_title(f"Complete mission: start -> slot {s.agent.target.sid} (reverse-in, nose-out)",
                 fontsize=9.5)
    save(fig, "fig_mission.png")

    # time histories
    t = np.array([r["t"] for r in log])
    fig, axs = plt.subplots(3, 1, figsize=(6.4, 5.6), sharex=True)
    axs[0].plot(t, [r["v_ref"] for r in log], color=C2, lw=1.4, label="reference $v_{ref}$")
    axs[0].plot(t, [r["v"] for r in log], color=C1, lw=1.6, label="actual $v$")
    axs[0].set_ylabel("speed [m/s]")
    axs[0].legend(loc="lower left", ncol=2, fontsize=8)
    axs[1].plot(t, np.degrees([r["steer"] for r in log]), color=C1, lw=1.6)
    axs[1].axhline(math.degrees(P.vehicle.max_steer), color=INK2, lw=0.8, ls=":")
    axs[1].axhline(-math.degrees(P.vehicle.max_steer), color=INK2, lw=0.8, ls=":")
    axs[1].set_ylabel("steering $\\delta$ [deg]")
    for law, col, lab in (("pure_pursuit", C1, "Pure Pursuit"), ("stanley", C2, "Stanley")):
        lg = runs[law].log
        axs[2].plot([r["t"] for r in lg], [r["cte"] * 100 for r in lg], color=col, lw=1.4, label=lab)
    axs[2].set_ylabel("cross-track [cm]")
    axs[2].set_xlabel("time [s]")
    axs[2].legend(loc="upper right", ncol=2, fontsize=8)
    states = [r["state"] for r in log]
    for ax in axs:
        for k in range(1, len(states)):
            if states[k] != states[k - 1]:
                ax.axvline(t[k], color=INK2, lw=0.7, ls="--")
    k = states.index("DOCK") if "DOCK" in states else None
    if k:
        axs[0].text(t[k] + 0.3, 0.9, "reverse + DOCK", fontsize=7.5, color=INK2)
    cusp = np.nonzero(np.diff(np.sign([r["v_ref"] for r in log])) != 0)[0]
    if len(cusp):
        axs[0].text(t[cusp[0]] - 0.3, 1.6, "gear change", fontsize=7.5, color=INK2, ha="right")
    save(fig, "fig_time_history.png")


# ---------------------------------------------------------------- figure 7
def fig_speed_loop():
    cp, tau = P.control, P.vehicle.speed_tau
    # Open loop L(s) = C(s) G(s); G(s) = 1 / (s (tau s + 1)) from accel command to speed...
    # here the plant from a_cmd to v is 1/(s (tau s + 1)); PID C = kd s + kp + ki/s
    num_c, den_c = [cp.speed_kd, cp.speed_kp, cp.speed_ki], [1, 0]
    num_g, den_g = [1], [tau, 1, 0]
    num_L, den_L = np.polymul(num_c, num_g), np.polymul(den_c, den_g)
    w = np.logspace(-2, 2, 600)
    _, H = signal.freqs(num_L, den_L, worN=w)
    mag, ph = 20 * np.log10(np.abs(H)), np.degrees(np.unwrap(np.angle(H)))
    i_c = np.argmin(np.abs(mag))
    pm = 180 + ph[i_c]
    wc = w[i_c]
    # closed-loop step response (derivative on measurement ignored -> conservative)
    den_cl = np.polyadd(den_L, num_L)
    t = np.linspace(0, 10, 2000)
    _, y = signal.step((num_L, den_cl), T=t)
    # simulate discrete loop with saturation, for comparison
    from valet.control.pid import PID
    pid, v, a, dt = PID(cp.speed_kp, cp.speed_ki, cp.speed_kd, cp.dt, -3.0, P.vehicle.max_accel), 0, 0, cp.dt
    td, vd = [], []
    for k in range(int(10 / dt)):
        u = pid(1.0, v)
        a += (u - a) * dt / tau
        v += a * dt
        td.append(k * dt); vd.append(v)
    vd = np.array(vd)

    def metrics(tt, yy):
        tr = tt[np.argmax(yy >= 0.9)] - tt[np.argmax(yy >= 0.1)]
        os_ = max(0.0, (yy.max() - 1) * 100)
        out = np.nonzero(np.abs(yy - 1) > 0.02)[0]
        ts = tt[out[-1]] if len(out) else 0.0
        return tr, os_, ts
    m_lin, m_dis = metrics(t, y), metrics(np.array(td), vd)

    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.0))
    axs[0].plot(t, y, color=C1, lw=1.6, label="linear model")
    axs[0].plot(td, vd, color=C2, lw=1.4, ls="--", label="discrete + saturation")
    axs[0].axhline(1.0, color=INK2, lw=0.7, ls=":")
    axs[0].set_xlabel("time [s]"); axs[0].set_ylabel("speed / setpoint")
    axs[0].set_title("(a) Unit step of the speed loop", fontsize=9.5)
    axs[0].legend(fontsize=7.5, loc="lower right")
    ax = axs[1]
    ax.semilogx(w, mag, color=C1, lw=1.6)
    ax.set_ylabel("|L(j$\\omega$)| [dB]", color=C1)
    ax.axhline(0, color=INK2, lw=0.7, ls=":")
    ax2 = ax.twinx()
    ax2.semilogx(w, ph, color=C2, lw=1.4, ls="--")
    ax2.set_ylabel("phase [deg]", color=C2)
    ax2.axhline(-180, color=INK2, lw=0.7, ls=":")
    ax2.spines["right"].set_visible(True)
    ax.set_xlabel("$\\omega$ [rad/s]")
    ax.set_title(f"(b) Bode of L(s): PM = {pm:.0f} deg at {wc:.2f} rad/s", fontsize=9.5)
    save(fig, "fig_speed_loop.png")
    return {"pm": pm, "wc": wc, "lin": m_lin, "dis": m_dis}


# -------------------------------------------------------------- figure 8
def fig_reflex_demo():
    out = {}
    for mode in ("fixed", "ttc"):
        s = Simulator(make_scenario(seed=1, occupancy=0.7, n_pedestrians=4), seed=1, reflex=mode)
        r = s.run()
        out[mode] = (s, r)
    fig, axs = plt.subplots(2, 1, figsize=(6.4, 4.4), sharex=True)
    for mode, col, lab in (("fixed", C2, "Fixed-distance stop"), ("ttc", C1, "Swept-footprint TTC")):
        s, r = out[mode]
        t = [x["t"] for x in s.log]
        axs[0].plot(t, [abs(x["v"]) for x in s.log], color=col, lw=1.5, label=lab)
        d = np.array([x["d_obs"] for x in s.log], float)
        d[~np.isfinite(d)] = np.nan
        axs[1].plot(t, d, color=col, lw=1.2, label=lab)
        if r["collision"]:
            axs[0].plot(t[-1], abs(s.log[-1]["v"]), "x", color=col, ms=9, mew=2)
            axs[0].annotate("contact", (t[-1], abs(s.log[-1]["v"])), (t[-1] + 1, 1.8), fontsize=8,
                            arrowprops=dict(arrowstyle="->", color=INK2, lw=0.8))
    axs[0].set_ylabel("|v| [m/s]")
    axs[0].legend(fontsize=8, loc="upper right")
    axs[1].axhline(P.reflex.stop_distance, color=INK2, ls=":", lw=0.8)
    axs[1].text(0.3, P.reflex.stop_distance + 0.15, "$d_{stop}$", fontsize=8, color=INK2)
    axs[1].set_ylabel("obstacle distance\nused by reflex [m]")
    axs[1].set_ylim(0, 6.5)
    axs[1].set_xlabel("time [s]")
    save(fig, "fig_reflex_timeline.png")


# ------------------------------------------------------- experiment figures
def load_rows():
    path = os.path.join(ROOT, "results", "experiments.csv")
    rows = list(csv.DictReader(open(path)))
    for r in rows:
        for k, v in r.items():
            if v in ("True", "False"):
                r[k] = v == "True"
            else:
                try:
                    r[k] = float(v)
                except (TypeError, ValueError):
                    pass
    g = defaultdict(list)
    for r in rows:
        g[(r["experiment"], r["label"])].append(r)
    return g


def agg(rs):
    def m(key, only_success=False):
        vals = [r[key] for r in rs if isinstance(r.get(key), float) and math.isfinite(r[key])
                and (r["success"] or not only_success)]
        return (st.mean(vals), st.pstdev(vals)) if vals else (float("nan"), float("nan"))
    n = len(rs)
    return {"n": n, "succ": 100 * sum(r["success"] for r in rs) / n,
            "coll": 100 * sum(r["collision"] for r in rs) / n,
            "plan": m("plan_time"), "exp": m("expansions"), "drive": m("drive_time", True),
            "len": m("path_length"), "gears": m("gear_switches"),
            "lat": tuple(abs(x) for x in m("lat_err", True)), "yaw": m("yaw_err_deg", True),
            "cte": m("mean_abs_cte", True), "maxcte": m("max_abs_cte", True),
            "clear": m("min_clearance"), "ped": m("min_ped_dist"), "cycle": m("cycle_length", True),
            "abslat": m_abs(rs, "lat_err"), "replans": m("replans")}


def m_abs(rs, key):
    vals = [abs(r[key]) for r in rs if r["success"] and isinstance(r.get(key), float)]
    return (st.mean(vals), st.pstdev(vals)) if vals else (float("nan"), float("nan"))


def bars(ax, labels, values, colors, fmt="{:.0f}", ylabel=""):
    x = np.arange(len(labels))
    ax.bar(x, values, color=colors, width=0.6, edgecolor="white", linewidth=2)
    for xi, v in zip(x, values):
        ax.text(xi, v, fmt.format(v), ha="center", va="bottom", fontsize=8, color=INK)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel(ylabel)
    ax.grid(axis="x", visible=False)


def fig_experiments(g):
    A = {k: agg(v) for k, v in g.items()}
    # E1 planners
    labs = ["Grid A*", "Hybrid A* (no funnel)", "Hybrid A* + funnel"]
    short = ["Grid A*", "Hybrid A*\n(no funnel)", "Hybrid A*\n+ funnel"]
    fig, axs = plt.subplots(1, 3, figsize=(7.2, 2.6))
    bars(axs[0], short, [A[("E1", l)]["succ"] for l in labs], [C2, C3, C1], "{:.0f}%", "success rate [%]")
    bars(axs[1], short, [A[("E1", l)]["plan"][0] for l in labs], [C2, C3, C1], "{:.2f}", "planning time [s]")
    bars(axs[2], short, [A[("E1", l)]["exp"][0] / 1000 for l in labs], [C2, C3, C1], "{:.1f}k",
         "node expansions [x1000]")
    for ax in axs:
        ax.tick_params(axis="x", labelsize=7)
    fig.tight_layout()
    save(fig, "fig_e1_planners.png")

    # E3 reflex
    labs = ["No reflex", "Fixed-distance", "Swept TTC"]
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.6))
    bars(axs[0], labs, [A[("E3", l)]["succ"] for l in labs], [C2, C3, C1], "{:.0f}%", "success rate [%]")
    bars(axs[1], labs, [A[("E3", l)]["coll"] for l in labs], [C2, C3, C1], "{:.0f}%", "missions with contact [%]")
    fig.tight_layout()
    save(fig, "fig_e3_reflex.png")

    # E5 noise
    levels = [0.0, 0.05, 0.10, 0.20, 0.30]
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.7))
    for pref, col, lab in (("raw", C2, "raw GPS"), ("KF", C1, "Kalman-filtered (ours)")):
        sr = [A[("E5", f"{pref} {g_:.2f}")]["succ"] for g_ in levels]
        le = [A[("E5", f"{pref} {g_:.2f}")]["abslat"][0] * 100 for g_ in levels]
        axs[0].plot(levels, sr, "o-", color=col, label=lab, ms=5)
        axs[1].plot(levels, le, "o-", color=col, label=lab, ms=5)
    axs[0].set_xlabel("GPS noise $\\sigma$ [m]"); axs[0].set_ylabel("success rate [%]")
    axs[0].set_ylim(-5, 105)
    axs[1].set_xlabel("GPS noise $\\sigma$ [m]"); axs[1].set_ylabel("final |lateral error| [cm]\n(successful runs)")
    axs[0].legend(fontsize=7.5, loc="lower left")
    fig.tight_layout()
    save(fig, "fig_e5_noise.png")

    # E6 occupancy + E4 cycle
    occ = ["50%", "70%", "85%", "92%"]
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.7))
    ax = axs[0]
    ax.plot([50, 70, 85, 92], [A[("E6", o)]["succ"] for o in occ], "o-", color=C1, ms=5, label="success [%]")
    ax.set_ylim(0, 105)
    ax.set_xlabel("lot occupancy [%]"); ax.set_ylabel("success rate [%]")
    ax.set_title("(a) Robustness to lot occupancy", fontsize=9.5)
    ax = axs[1]
    data = [[r["cycle_length"] for r in g[("E4", l)] if isinstance(r.get("cycle_length"), float)]
            for l in ("Nearest slot", "Utility agent")]
    bp = ax.boxplot(data, widths=0.5, patch_artist=True, medianprops=dict(color=INK))
    for patch, col in zip(bp["boxes"], (C2, C1)):
        patch.set_facecolor(col); patch.set_alpha(0.55); patch.set_edgecolor(col)
    ax.set_xticks([1, 2]); ax.set_xticklabels(["Nearest slot", "Utility agent"])
    ax.set_ylabel("valet cycle length [m]\n(drive in + drive out)")
    ax.set_title("(b) Slot-selection policy (85 % full)", fontsize=9.5)
    fig.tight_layout()
    save(fig, "fig_e4_e6.png")

    # final pose accuracy histogram (all successful Hybrid+PP runs in E1/E2/E6)
    lat, yaw = [], []
    for key in (("E1", "Hybrid A* + funnel"), ("E2", "Pure Pursuit"), ("E6", "70%"), ("E6", "50%")):
        for r in g.get(key, []):
            if r["success"]:
                lat.append(r["lat_err"] * 100); yaw.append(r["yaw_err_deg"])
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.5))
    axs[0].hist(lat, bins=16, color=C1, edgecolor="white")
    axs[0].set_xlabel("final lateral error [cm]"); axs[0].set_ylabel("missions")
    axs[1].hist(yaw, bins=16, color=C3, edgecolor="white")
    axs[1].set_xlabel("final heading error [deg]")
    fig.tight_layout()
    save(fig, "fig_final_errors.png")
    return A


# ------------------------------------------------------------------- tables
def f(m, d=2):
    return "--" if not math.isfinite(m[0]) else f"{m[0]:.{d}f} $\\pm$ {m[1]:.{d}f}"


def write_tables(A, extra):
    def tab(name, header, rows):
        with open(os.path.join(TAB, name), "w") as fh:
            fh.write(" & ".join(header) + " \\\\ \\hline\n")
            for r in rows:
                fh.write(" & ".join(r) + " \\\\ \\hline\n")
        print("  table", name)

    labs = ["Grid A*", "Hybrid A* (no funnel)", "Hybrid A* + funnel"]
    tab("e1.tex", ["\\textbf{Metric}"] + [f"\\textbf{{{l}}}" for l in labs], [
        ["Missions (seeds)"] + [f"{A[('E1', l)]['n']}" for l in labs],
        ["Success rate (\\%)"] + [f"{A[('E1', l)]['succ']:.1f}" for l in labs],
        ["Collisions (\\%)"] + [f"{A[('E1', l)]['coll']:.1f}" for l in labs],
        ["Planning time (s)"] + [f(A[("E1", l)]["plan"]) for l in labs],
        ["Node expansions"] + [f(A[("E1", l)]["exp"], 0) for l in labs],
        ["Path length (m)"] + [f(A[("E1", l)]["len"], 1) for l in labs],
        ["Gear changes"] + [f(A[("E1", l)]["gears"], 2) for l in labs],
        ["Drive time, successful (s)"] + [f(A[("E1", l)]["drive"], 1) for l in labs]])
    labs = ["Pure Pursuit", "Stanley"]
    tab("e2.tex", ["\\textbf{Metric}"] + [f"\\textbf{{{l}}}" for l in labs], [
        ["Success rate (\\%)"] + [f"{A[('E2', l)]['succ']:.1f}" for l in labs],
        ["Mean $|e_{ct}|$ (cm)"] + [f((A[("E2", l)]["cte"][0] * 100, A[("E2", l)]["cte"][1] * 100), 1) for l in labs],
        ["Max $|e_{ct}|$ (cm)"] + [f((A[("E2", l)]["maxcte"][0] * 100, A[("E2", l)]["maxcte"][1] * 100), 1) for l in labs],
        ["Final $|$lateral$|$ (cm)"] + [f((A[("E2", l)]["abslat"][0] * 100, A[("E2", l)]["abslat"][1] * 100), 1) for l in labs],
        ["Final heading error (deg)"] + [f(A[("E2", l)]["yaw"], 2) for l in labs],
        ["Drive time (s)"] + [f(A[("E2", l)]["drive"], 1) for l in labs]])
    labs = ["No reflex", "Fixed-distance", "Swept TTC"]
    tab("e3.tex", ["\\textbf{Metric}"] + [f"\\textbf{{{l}}}" for l in labs], [
        ["Success rate (\\%)"] + [f"{A[('E3', l)]['succ']:.1f}" for l in labs],
        ["Missions with contact (\\%)"] + [f"{A[('E3', l)]['coll']:.1f}" for l in labs],
        ["Min. pedestrian gap (m)"] + [f(A[("E3", l)]["ped"], 2) for l in labs],
        ["Re-plans per mission"] + [f(A[("E3", l)]["replans"], 2) for l in labs],
        ["Drive time, successful (s)"] + [f(A[("E3", l)]["drive"], 1) for l in labs]])
    labs = ["Nearest slot", "Utility agent"]
    tab("e4.tex", ["\\textbf{Metric}"] + [f"\\textbf{{{l}}}" for l in labs], [
        ["Success rate (\\%)"] + [f"{A[('E4', l)]['succ']:.1f}" for l in labs],
        ["Valet cycle length (m)"] + [f(A[("E4", l)]["cycle"], 1) for l in labs],
        ["Parking path length (m)"] + [f(A[("E4", l)]["len"], 1) for l in labs],
        ["Planning time (s)"] + [f(A[("E4", l)]["plan"], 2) for l in labs],
        ["Drive time (s)"] + [f(A[("E4", l)]["drive"], 1) for l in labs]])
    rows = []
    for lev in (0.0, 0.05, 0.10, 0.20, 0.30):
        ra, ka = A[("E5", f"raw {lev:.2f}")], A[("E5", f"KF {lev:.2f}")]
        rows.append([f"{lev:.2f}", f"{ra['succ']:.0f}", f"{ka['succ']:.0f}",
                     f"{ra['abslat'][0] * 100:.1f}" if math.isfinite(ra['abslat'][0]) else "--",
                     f"{ka['abslat'][0] * 100:.1f}"])
    tab("e5.tex", ["\\textbf{GPS $\\sigma$ (m)}", "\\textbf{Success raw (\\%)}", "\\textbf{Success KF (\\%)}",
                   "\\textbf{$|$lat$|$ raw (cm)}", "\\textbf{$|$lat$|$ KF (cm)}"], rows)
    rows = []
    for o in ("50%", "70%", "85%", "92%"):
        a = A[("E6", o)]
        rows.append([o.replace("%", "\\%"), f"{a['n']}", f"{a['succ']:.0f}", f(a["plan"], 2), f(a["drive"], 1)])
    e7 = A[("E7", "Slot taken")]
    rows.append(["70\\% + slot taken", f"{e7['n']}", f"{e7['succ']:.0f}", f(e7["plan"], 2), f(e7["drive"], 1)])
    tab("e6.tex", ["\\textbf{Scenario}", "\\textbf{Runs}", "\\textbf{Success (\\%)}",
                   "\\textbf{Planning time (s)}", "\\textbf{Drive time (s)}"], rows)

    # LaTeX macros so that every number quoted in the report text comes from the data
    def pct(k):
        return f"{A[k]['succ']:.0f}"
    e1h, e1n, e1g = A[("E1", "Hybrid A* + funnel")], A[("E1", "Hybrid A* (no funnel)")], A[("E1", "Grid A*")]
    e4n, e4u = A[("E4", "Nearest slot")], A[("E4", "Utility agent")]
    sl = extra["speed"]
    macros = {
        "NMissions": f"{sum(a['n'] for a in A.values())}",
        "EOneSuccGrid": pct(("E1", "Grid A*")), "EOneCollGrid": f"{e1g['coll']:.0f}",
        "EOneSuccNoFunnel": pct(("E1", "Hybrid A* (no funnel)")), "EOneSuccFunnel": pct(("E1", "Hybrid A* + funnel")),
        "EOnePlanNoFunnel": f"{e1n['plan'][0]:.1f}", "EOnePlanFunnel": f"{e1h['plan'][0]:.1f}",
        "EOneExpNoFunnel": f"{e1n['exp'][0]:,.0f}".replace(",", "{,}"), "EOneExpFunnel": f"{e1h['exp'][0]:,.0f}".replace(",", "{,}"),
        "EOneExpRatio": f"{e1n['exp'][0] / e1h['exp'][0]:.1f}", "EOneTimeRatio": f"{e1n['plan'][0] / e1h['plan'][0]:.1f}",
        "EOneLenFunnel": f"{e1h['len'][0]:.1f}", "EOneGearsFunnel": f"{e1h['gears'][0]:.2f}",
        "EOneDriveFunnel": f"{e1h['drive'][0]:.1f}",
        "ETwoSuccPP": pct(("E2", "Pure Pursuit")), "ETwoSuccST": pct(("E2", "Stanley")),
        "ETwoCtePP": f"{A[('E2', 'Pure Pursuit')]['cte'][0] * 100:.1f}", "ETwoCteST": f"{A[('E2', 'Stanley')]['cte'][0] * 100:.1f}",
        "ETwoLatPP": f"{A[('E2', 'Pure Pursuit')]['abslat'][0] * 100:.1f}", "ETwoLatST": f"{A[('E2', 'Stanley')]['abslat'][0] * 100:.1f}",
        "ETwoYawPP": f"{A[('E2', 'Pure Pursuit')]['yaw'][0]:.1f}", "ETwoYawST": f"{A[('E2', 'Stanley')]['yaw'][0]:.1f}",
        "EThreeSuccNone": pct(("E3", "No reflex")), "EThreeSuccFixed": pct(("E3", "Fixed-distance")),
        "EThreeSuccTTC": pct(("E3", "Swept TTC")), "EThreeCollNone": f"{A[('E3', 'No reflex')]['coll']:.0f}",
        "EThreeCollFixed": f"{A[('E3', 'Fixed-distance')]['coll']:.0f}", "EThreeCollTTC": f"{A[('E3', 'Swept TTC')]['coll']:.0f}",
        "EThreeDriveFixed": f"{A[('E3', 'Fixed-distance')]['drive'][0]:.1f}", "EThreeDriveTTC": f"{A[('E3', 'Swept TTC')]['drive'][0]:.1f}",
        "EFourSuccNear": f"{e4n['succ']:.0f}", "EFourSuccUtil": f"{e4u['succ']:.0f}",
        "EFourCycleNear": f"{e4n['cycle'][0]:.1f}", "EFourCycleUtil": f"{e4u['cycle'][0]:.1f}",
        "EFourCycleGain": f"{100 * (1 - e4u['cycle'][0] / e4n['cycle'][0]):.1f}",
        "EFiveRawTwo": pct(("E5", "raw 0.20")), "EFiveKFTwo": pct(("E5", "KF 0.20")),
        "EFiveRawThree": pct(("E5", "raw 0.30")), "EFiveKFThree": pct(("E5", "KF 0.30")),
        "ESixSuccFifty": pct(("E6", "50%")), "ESixSuccNinetyTwo": pct(("E6", "92%")),
        "ESixSuccEightyFive": pct(("E6", "85%")), "ESevenSucc": pct(("E7", "Slot taken")),
        "PhaseMargin": f"{sl['pm']:.0f}", "CrossFreq": f"{sl['wc']:.2f}",
        "RiseLin": f"{sl['lin'][0]:.2f}", "OvershootLin": f"{sl['lin'][1]:.0f}", "SettleLin": f"{sl['lin'][2]:.1f}",
        "RiseDis": f"{sl['dis'][0]:.2f}", "OvershootDis": f"{sl['dis'][1]:.0f}", "SettleDis": f"{sl['dis'][2]:.1f}",
    }
    with open(os.path.join(TAB, "numbers.tex"), "w") as fh:
        for k, v in macros.items():
            fh.write(f"\\newcommand{{\\{k}}}{{{v}}}\n")
    print("  table numbers.tex")

    # human-readable summary for the team
    with open(os.path.join(ROOT, "results", "summary.md"), "w") as fh:
        fh.write("# Experiment summary (auto-generated by make_figures.py)\n\n")
        for k in sorted(A):
            a = A[k]
            fh.write(f"- **{k[0]} / {k[1]}** (n={a['n']}): success {a['succ']:.1f}%, contact {a['coll']:.1f}%, "
                     f"plan {a['plan'][0]:.2f}s, expansions {a['exp'][0]:.0f}, drive {a['drive'][0]:.1f}s, "
                     f"|lat| {a['abslat'][0]*100:.1f}cm, yaw {a['yaw'][0]:.2f}deg, cycle {a['cycle'][0]:.1f}m\n")
        sl = extra["speed"]
        fh.write(f"\nSpeed loop: phase margin {sl['pm']:.1f} deg at {sl['wc']:.2f} rad/s; "
                 f"linear step: rise {sl['lin'][0]:.2f}s, overshoot {sl['lin'][1]:.1f}%, settle(2%) {sl['lin'][2]:.2f}s; "
                 f"discrete+saturation: rise {sl['dis'][0]:.2f}s, overshoot {sl['dis'][1]:.1f}%, settle {sl['dis'][2]:.2f}s\n")
        h, n, gi = extra["plan_demo"]
        fh.write(f"Planner demo (seed 1, slot B3): funnel {h['expansions']} exp / {h['time']:.2f}s; "
                 f"no funnel {n['expansions']} exp / {n['time']:.2f}s; grid {gi['expansions']} exp / {gi['time']*1000:.1f}ms\n")
    print("  summary.md")


def main():
    print("figures ...")
    fig_lot_and_sensors()
    plan_demo = fig_planner_comparison()
    runs = demo_runs()
    fig_mission(runs)
    speed = fig_speed_loop()
    fig_reflex_demo()
    g = load_rows()
    A = fig_experiments(g)
    write_tables(A, {"speed": speed, "plan_demo": plan_demo})


if __name__ == "__main__":
    main()
