"""Monte Carlo experiments for the report (Section 9).

    python experiments/run_experiments.py            # all experiments (~10-25 min)
    python experiments/run_experiments.py --quick    # 5 seeds each, smoke test

Every run uses a fixed seed, so the numbers in the report are reproducible.
Results -> results/experiments.csv (one row per mission).

  E1  planner        grid A* | Hybrid A* (no funnel) | Hybrid A* + goal funnel
  E2  tracking law   Pure Pursuit | Stanley
  E3  reflex layer   none | fixed-distance stop | swept-footprint TTC (4 pedestrians)
  E4  slot choice    nearest free slot | utility-based agent (85 % occupancy)
  E5  GPS noise      sigma = 0 ... 0.30 m, raw GPS vs Kalman-filtered pose
  E6  occupancy      50, 70, 85, 92 %
  E7  dynamic event  target slot taken by another car during the approach
"""
import argparse
import csv
import os
import sys
import time
from multiprocessing import Pool

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from valet.lot import make_scenario      # noqa: E402
from sim.simulator import Simulator      # noqa: E402


def job(spec):
    exp, label, seed, kw, sc_kw = spec
    sc = make_scenario(seed=seed, **sc_kw)
    t0 = time.perf_counter()
    r = Simulator(sc, seed=seed, **kw).run(record=True)
    r.update(experiment=exp, label=label, wall_time=time.perf_counter() - t0,
             occupancy=sc_kw.get("occupancy", 0.7), gps_noise=kw.get("gps_noise", 0.05),
             n_peds=sc_kw.get("n_pedestrians", 0))
    return r


def build_jobs(n):
    seeds = list(range(100, 100 + n))
    J = []
    for s in seeds:
        for lab, pl in (("Grid A*", "grid"), ("Hybrid A* (no funnel)", "hybrid_nofunnel"),
                        ("Hybrid A* + funnel", "hybrid")):
            J.append(("E1", lab, s, {"planner": pl}, {"occupancy": 0.7}))
        for lab, law in (("Pure Pursuit", "pure_pursuit"), ("Stanley", "stanley")):
            J.append(("E2", lab, s, {"law": law}, {"occupancy": 0.7}))
        for lab, rf in (("No reflex", "none"), ("Fixed-distance", "fixed"), ("Swept TTC", "ttc")):
            J.append(("E3", lab, s, {"reflex": rf}, {"occupancy": 0.7, "n_pedestrians": 4}))
        for lab, sel in (("Nearest slot", "nearest"), ("Utility agent", "utility")):
            J.append(("E4", lab, s, {"selector": sel}, {"occupancy": 0.85}))
    for s in seeds[: max(n // 2, 5)]:   # E5-E7 use half the seeds (more settings)
        for g in (0.0, 0.05, 0.10, 0.20, 0.30):
            J.append(("E5", f"KF {g:.2f}", s, {"gps_noise": g}, {"occupancy": 0.7}))
            J.append(("E5", f"raw {g:.2f}", s, {"gps_noise": g, "localisation": False},
                      {"occupancy": 0.7}))
        for occ in (0.5, 0.7, 0.85, 0.92):
            J.append(("E6", f"{int(occ * 100)}%", s, {}, {"occupancy": occ}))
        J.append(("E7", "Slot taken", s, {}, {"occupancy": 0.7, "steal": True}))
    return J


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--n", type=int, default=30, help="seeds per configuration")
    ap.add_argument("--workers", type=int, default=max(os.cpu_count() - 1, 1))
    a = ap.parse_args()
    n = 5 if a.quick else a.n
    jobs = build_jobs(n)
    print(f"{len(jobs)} missions on {a.workers} workers ...")
    t0 = time.time()
    rows = []
    with Pool(a.workers) as pool:
        for k, r in enumerate(pool.imap_unordered(job, jobs, chunksize=1), 1):
            rows.append(r)
            if k % 25 == 0 or k == len(jobs):
                print(f"  {k}/{len(jobs)}  ({time.time() - t0:.0f} s)", flush=True)
    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    out = os.path.join(ROOT, "results", "experiments.csv")
    keys = sorted({k for r in rows for k in r})
    rows.sort(key=lambda r: (r["experiment"], r["label"], r["seed"]))
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {out} in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
