"""Webots controller for the G14 Autonomous Valet Parking Agent.

Runs the SAME three-layer agent (valet/agent.py) that the headless twin
uses, on the BmwX5 Ackermann car of the Webots vehicle library.

Devices (created by tools/make_world.py in the BmwX5 sensorsSlotCenter,
whose origin is the centre of the rear axle):
    gps, imu, 26 DistanceSensors (names in valet/config.py)

Controller arguments (set in the world file, see tools/make_world.py):
    --planner=hybrid|grid  --law=pure_pursuit|stanley
    --reflex=ttc|fixed|none  --selector=utility|nearest
    --speed-mode=cruise|throttle  --steer-sign=-1|1

Webots convention: Driver.setSteeringAngle() takes positive angles for a
RIGHT turn, the agent uses positive = LEFT (ISO 8855), hence steer_sign = -1.
If the car turns the wrong way at the first bend, run with --steer-sign=1.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)

from vehicle import Driver                       # noqa: E402  (Webots library)

from valet.config import DEFAULT                 # noqa: E402
from valet.geometry import OBB                   # noqa: E402
from valet.lot import build_slots                # noqa: E402
from valet.agent import ValetAgent               # noqa: E402
from valet.control.pid import PID                # noqa: E402


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--planner", default="hybrid")
    ap.add_argument("--law", default="pure_pursuit")
    ap.add_argument("--reflex", default="ttc")
    ap.add_argument("--selector", default="utility")
    ap.add_argument("--speed-mode", default="cruise", choices=["cruise", "throttle"])
    ap.add_argument("--steer-sign", type=float, default=-1.0)
    return ap.parse_args(sys.argv[1:])


def load_infrastructure(p):
    """Slot occupancy + parked-car boxes, as a smart-parking system would send them."""
    with open(os.path.join(ROOT, "worlds", "lot_config.json")) as f:
        cfg = json.load(f)
    slots = build_slots(p)
    occ = set(cfg["occupied"])
    for s in slots:
        s.occupied = s.sid in occ
    boxes = [OBB(*w, kind="wall") for w in cfg["walls"]] + [OBB(*b, kind="car") for b in cfg["parked"]]
    return slots, boxes, tuple(cfg["exit_point"]), cfg


def main():
    a = parse_args()
    p = DEFAULT
    driver = Driver()
    basic = int(driver.getBasicTimeStep())
    every = max(1, round(p.control.dt * 1000 / basic))         # control period ~ 48 ms
    dt = every * basic / 1000.0

    gps = driver.getDevice("gps"); gps.enable(basic)
    imu = driver.getDevice("imu"); imu.enable(basic)
    sensors = {}
    for name, *_ in p.sensors.mounts:
        dev = driver.getDevice(name)
        if dev is not None:
            dev.enable(basic)
            sensors[name] = dev

    slots, boxes, exit_point, cfg = load_infrastructure(p)
    agent = ValetAgent(slots, boxes, exit_point, p, a.planner, a.law, a.reflex, a.selector)
    throttle_pid = PID(0.35, 0.15, 0.0, dt, -1.0, 1.0)

    os.makedirs(os.path.join(ROOT, "results", "webots_runs"), exist_ok=True)
    log_path = os.path.join(ROOT, "results", "webots_runs",
                            time.strftime(f"run_seed{cfg['seed']}_%Y%m%d_%H%M%S.csv"))
    log_f = open(log_path, "w", newline="")
    log = csv.writer(log_f)
    log.writerow(["t", "x", "y", "yaw", "v", "v_ref", "steer", "state", "rule", "cte", "d_obs"])

    # Self-check: a sensor stuck near 0 m usually sees the car's own body ->
    # move it a few cm outwards in valet/config.py and regenerate the world.
    for _ in range(3):
        driver.step()
    missing = [n for n, *_ in p.sensors.mounts if n not in sensors]
    stuck = [n for n, d in sensors.items() if d.getValue() < 0.05]
    if missing:
        print(f"[valet] WARNING: sensors not found in the world: {missing}")
    if stuck:
        print(f"[valet] WARNING: sensors reading ~0 m at start (self-hit?): {stuck}")

    prev_xy, v_est, step, last_state = None, 0.0, 0, None
    print(f"[valet] planner={a.planner} law={a.law} reflex={a.reflex} selector={a.selector}")
    while driver.step() != -1:
        step += 1
        if step % every:
            continue
        t = driver.getTime()
        x, y, _ = gps.getValues()
        yaw = imu.getRollPitchYaw()[2]
        if any(math.isnan(c) for c in (x, y, yaw)):
            continue
        # signed speed from GPS displacement projected on the heading
        if prev_xy is not None:
            dx, dy = x - prev_xy[0], y - prev_xy[1]
            v_est = 0.6 * v_est + 0.4 * (dx * math.cos(yaw) + dy * math.sin(yaw)) / dt
        prev_xy = (x, y)
        ranges = {n: d.getValue() for n, d in sensors.items()}

        cmd = agent.step(t, x, y, yaw, v_est, ranges, dt)
        if cmd.state != last_state:
            tgt = agent.target.sid if agent.target else "-"
            print(f"[valet] t={t:6.2f}s  state -> {cmd.state}  (slot {tgt})")
            if cmd.state == "DRIVE" and agent.path is not None:
                print(f"[valet]   path {agent.path.length:.1f} m, {agent.path.n_gear_switches} gear "
                      f"change(s), planned in {agent.stats['plan_time']:.2f} s")
            last_state = cmd.state

        driver.setSteeringAngle(a.steer_sign * cmd.steer)
        if cmd.state in ("PARKED", "FAILED"):
            driver.setCruisingSpeed(0.0)
            driver.setBrakeIntensity(1.0)
        elif a.speed_mode == "cruise":
            driver.setCruisingSpeed(cmd.v_ref * 3.6)              # km/h, negative = reverse
        else:
            driver.setGear(1 if cmd.v_ref >= 0 else -1)
            u = throttle_pid(abs(cmd.v_ref), abs(v_est))
            driver.setThrottle(max(u, 0.0))
            driver.setBrakeIntensity(max(-u, 0.0))

        log.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:.3f}", f"{yaw:.4f}", f"{v_est:.3f}",
                      f"{cmd.v_ref:.3f}", f"{cmd.steer:.4f}", cmd.state, cmd.info.get("rule", "-"),
                      f"{cmd.info.get('cte', float('nan')):.3f}", f"{cmd.info.get('d_obs', float('nan')):.3f}"])
        if cmd.state in ("PARKED", "FAILED") and abs(v_est) < 0.05:
            print(f"[valet] mission finished: {cmd.state} in slot "
                  f"{agent.target.sid if agent.target else '-'} after {t:.1f} s; log -> {log_path}")
            break
    log_f.close()
    return driver, agent


if __name__ == "__main__":
    drv, _ = main()
    # keep the simulation running (car parked, brakes on) so the demo can be inspected
    while drv.step() != -1:
        pass
