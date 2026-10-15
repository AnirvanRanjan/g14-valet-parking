"""Run one mission in the headless twin and save an animation + a snapshot.

    python sim/run_demo.py --seed 3 --pedestrians 2           # -> results/demo.gif
    python sim/run_demo.py --seed 3 --steal --out results/demo_steal.gif
"""
import argparse
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
import matplotlib.patches as mp                       # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter   # noqa: E402

from valet.config import DEFAULT                      # noqa: E402
from valet.geometry import car_footprint              # noqa: E402
from valet.lot import make_scenario                   # noqa: E402
from sim.simulator import Simulator                   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--occupancy", type=float, default=0.7)
    ap.add_argument("--pedestrians", type=int, default=2)
    ap.add_argument("--steal", action="store_true")
    ap.add_argument("--planner", default="hybrid")
    ap.add_argument("--law", default="pure_pursuit")
    ap.add_argument("--reflex", default="ttc")
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "demo.gif"))
    ap.add_argument("--every", type=int, default=4, help="simulation steps per frame")
    a = ap.parse_args()

    sc = make_scenario(a.seed, a.occupancy, a.pedestrians, a.steal)
    sim = Simulator(sc, seed=a.seed, planner=a.planner, law=a.law, reflex=a.reflex)
    res = sim.run()
    print({k: res[k] for k in ("state", "success", "slot", "drive_time", "plan_time", "collision")})
    log = sim.log[::a.every]

    fig, ax = plt.subplots(figsize=(7, 6.4))
    lp = DEFAULT.lot
    for s in sc.slots:
        ax.add_patch(mp.Polygon(s.rect().corners(), fill=False, ec="#9a9994", lw=0.5))
    for b in sim.obstacles:
        ax.add_patch(mp.Polygon(b.corners(), fc="#6b6a66" if b.kind == "wall" else "#b9b8b2", ec="none"))
    if sim.agent.path is not None:
        ax.plot(sim.agent.path.x, sim.agent.path.y, "--", color="#52514e", lw=1)
    ax.set_aspect("equal"); ax.set_xlim(-1, lp.width + 1); ax.set_ylim(-1, lp.height + 1)
    car_patch = mp.Polygon(car_footprint(*sc.start, DEFAULT.vehicle).corners(), fc="#2a78d6", ec="k", lw=0.8)
    ax.add_patch(car_patch)
    trail, = ax.plot([], [], color="#2a78d6", lw=1.5)
    peds = [mp.Circle((0, 0), 0.35, fc="#eb6834") for _ in sc.pedestrians]
    for c in peds:
        ax.add_patch(c)
    title = ax.set_title("")
    xs, ys = [], []

    def update(k):
        r = log[k]
        car_patch.set_xy(car_footprint(r["x"], r["y"], r["yaw"], DEFAULT.vehicle).corners())
        car_patch.set_facecolor("#2a78d6" if r["v"] >= 0 else "#eb6834")
        xs.append(r["x"]); ys.append(r["y"])
        trail.set_data(xs, ys)
        for c, pp in zip(peds, r["peds"]):
            c.center = pp
        title.set_text(f"t = {r['t']:5.1f} s   state: {r['state']:6s}   v = {r['v']:+.2f} m/s   "
                       f"reflex: {r['rule']}")
        return [car_patch, trail, title] + peds

    anim = FuncAnimation(fig, update, frames=len(log), blit=False)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    anim.save(a.out, writer=PillowWriter(fps=int(1 / (DEFAULT.control.dt * a.every))))
    fig.savefig(a.out.replace(".gif", "_final.png"), dpi=150)
    print("saved", a.out)


if __name__ == "__main__":
    main()
