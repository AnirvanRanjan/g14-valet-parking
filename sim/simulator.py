"""Headless Python twin of the Webots parking-lot world.

Used for batch (Monte Carlo) experiments, because a full Webots run of
hundreds of missions is slow. The twin uses the same lot geometry, vehicle
dimensions and sensor layout as worlds/valet_parking.wbt, and the SAME agent
code (valet/agent.py) that runs inside the Webots controller.

Vehicle model: kinematic bicycle (rear-axle reference) with
  * steering-rate limit                     |d delta/dt| <= max_steer_rate
  * first-order drivetrain lag              d a/dt = (a_cmd - a) / tau
  * speed limits and actuator saturation.
Sensors: GPS (Gaussian noise), compass/IMU yaw (noise), wheel-speed
(noise), eight ray-cast range sensors (noise, max range).
"""
import math
import random
import numpy as np

from valet.config import DEFAULT
from valet.geometry import (OBB, car_footprint, obb_overlap, circle_obb_distance,
                            ray_cast_boxes, ray_cast_circles, wrap_angle)
from valet.lot import parked_car_box
from valet.agent import ValetAgent


class CarModel:
    def __init__(self, pose, p=DEFAULT):
        self.x, self.y, self.yaw = pose
        self.v, self.a, self.delta = 0.0, 0.0, 0.0
        self.p = p

    def step(self, steer_cmd, accel_cmd, dt):
        vp = self.p.vehicle
        steer_cmd = max(-vp.max_steer, min(vp.max_steer, steer_cmd))
        dmax = vp.max_steer_rate * dt
        self.delta += max(-dmax, min(dmax, steer_cmd - self.delta))
        accel_cmd = max(-3.0, min(vp.max_accel, accel_cmd))
        self.a += (accel_cmd - self.a) * dt / vp.speed_tau
        self.v = max(-vp.max_reverse_speed, min(vp.max_speed, self.v + self.a * dt))
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = wrap_angle(self.yaw + self.v * math.tan(self.delta) / vp.wheelbase * dt)


class Simulator:
    def __init__(self, scenario, p=DEFAULT, planner="hybrid", law="pure_pursuit",
                 reflex="ttc", selector="utility", noise=True, gps_noise=None, seed=0,
                 max_time=240.0, localisation=True):
        self.sc, self.p = scenario, p
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.gps_noise = p.sensors.gps_noise_std if gps_noise is None else gps_noise
        self.car = CarModel(scenario.start, p)
        self.t = 0.0
        self.max_time = max_time
        self.obstacles = list(scenario.obstacles)
        # The agent gets the lot map + parked-car boxes from the "infrastructure".
        agent_slots = [type(s)(**vars(s)) for s in scenario.slots]
        self.agent = ValetAgent(agent_slots, scenario.obstacles, scenario.exit_point, p,
                                planner, law, reflex, selector, localisation=localisation)
        self.log = []
        self.collided = False
        self.collision_with = None
        self.min_static_clearance = math.inf
        self.min_ped_distance = math.inf
        self.steal_done = False
        mounts = p.sensors.mounts
        self.m_names = [m[0] for m in mounts]
        self.m_xy = np.array([[m[1], m[2]] for m in mounts])
        self.m_yaw = np.array([m[3] for m in mounts])

    # ------------------------------------------------------------------ sensors
    def ped_positions(self):
        return [ped.position() for ped in self.sc.pedestrians]

    def _ped_blocked(self, nxt, cur):
        """Pedestrian courtesy rule: do not step closer to a car that is within 0.6 m."""
        fp = car_footprint(self.car.x, self.car.y, self.car.yaw, self.p.vehicle)
        dn = circle_obb_distance(nxt[0], nxt[1], fp)
        return dn < 0.35 + 0.6 and dn < circle_obb_distance(cur[0], cur[1], fp)

    def ranges(self):
        c, s = math.cos(self.car.yaw), math.sin(self.car.yaw)
        ox = self.car.x + c * self.m_xy[:, 0] - s * self.m_xy[:, 1]
        oy = self.car.y + s * self.m_xy[:, 0] + c * self.m_xy[:, 1]
        ang = self.car.yaw + self.m_yaw
        dirs = np.c_[np.cos(ang), np.sin(ang)]
        mr = self.p.sensors.max_range
        near = [b for b in self.obstacles if abs(b.cx - self.car.x) < 14 and abs(b.cy - self.car.y) < 14]
        r = ray_cast_boxes(ox, oy, dirs, near, mr)
        peds = self.ped_positions()
        if peds:
            r = np.minimum(r, ray_cast_circles(ox, oy, dirs, peds, 0.35, mr))
        if self.noise:
            r = np.where(r < mr, r + self.rng.normal(0, self.p.sensors.noise_std, len(r)), r)
        r = np.clip(r, 0.0, mr)
        return dict(zip(self.m_names, r.tolist()))

    def measured_pose(self):
        if not self.noise:
            return self.car.x, self.car.y, self.car.yaw, self.car.v
        sp = self.p.sensors
        return (self.car.x + self.rng.normal(0, self.gps_noise),
                self.car.y + self.rng.normal(0, self.gps_noise),
                wrap_angle(self.car.yaw + self.rng.normal(0, sp.heading_noise_std)),
                self.car.v + self.rng.normal(0, 0.02))

    # ----------------------------------------------------------- ground truth
    def check_collision(self):
        fp = car_footprint(self.car.x, self.car.y, self.car.yaw, self.p.vehicle)
        clear = float(self.agent.costmap.pose_clearance(self.car.x, self.car.y, self.car.yaw)[0]) \
            if not self.agent.temp_boxes else math.inf
        self.min_static_clearance = min(self.min_static_clearance, clear)
        for b in self.obstacles:
            if abs(b.cx - fp.cx) < 8 and abs(b.cy - fp.cy) < 8 and obb_overlap(fp, b):
                return b.kind
        for (px, py) in self.ped_positions():
            d = circle_obb_distance(px, py, fp) - 0.35
            self.min_ped_distance = min(self.min_ped_distance, d)
            if d <= 0:
                return "pedestrian"
        return None

    def _steal(self):
        """Another car takes the agent's target slot (dynamic event)."""
        tgt = self.agent.target
        if tgt is None or self.agent.state in ("PARKED", "FAILED"):
            return
        slot = self.sc.slot(tgt.sid)
        slot.occupied = True
        box = parked_car_box(slot, random.Random(self.sc.seed + 99), self.p)
        self.obstacles.append(box)
        self.agent.notify_occupied(tgt.sid, box)
        self.steal_done = True

    # --------------------------------------------------------------------- run
    def run(self, record=True):
        dt = self.p.control.dt
        while self.t < self.max_time:
            if self.sc.steal_event and not self.steal_done and self.t >= self.sc.steal_event[0]:
                self._steal()
            mx, my, myaw, mv = self.measured_pose()
            rng = self.ranges()
            cmd = self.agent.step(self.t, mx, my, myaw, mv, rng, dt)
            if cmd.state in ("PARKED", "FAILED") and abs(self.car.v) < 0.05:
                break
            self.car.step(cmd.steer, cmd.accel, dt)
            for ped in self.sc.pedestrians:
                ped.step(self.t, dt, self._ped_blocked)
            hit = self.check_collision()
            if record:
                self.log.append({"t": self.t, "x": self.car.x, "y": self.car.y, "yaw": self.car.yaw,
                                 "v": self.car.v, "v_ref": cmd.v_ref, "steer": self.car.delta,
                                 "state": cmd.state, "cte": cmd.info.get("cte", float("nan")),
                                 "yaw_err": cmd.info.get("yaw_err", float("nan")),
                                 "rule": cmd.info.get("rule", "-"), "d_obs": cmd.info.get("d_obs", float("nan")),
                                 "peds": self.ped_positions()})
            if hit:
                self.collided, self.collision_with = True, hit
                break
            self.t += dt
        return self.result()

    def result(self):
        ag = self.agent
        res = {"seed": self.sc.seed, "planner": ag.planner_name, "law": ag.tracker.law.name,
               "reflex": ag.reflex.mode, "selector": ag.selector_name, "state": ag.state,
               "localisation": ag.pose_filter is not None,
               "collision": self.collided, "collision_with": self.collision_with,
               "drive_time": self.t, "plan_time": ag.stats["plan_time"],
               "expansions": ag.stats["expansions"], "plans": ag.stats["plans"],
               "replans": ag.stats["replans"], "reselections": ag.stats["reselections"],
               "min_clearance": self.min_static_clearance, "min_ped_dist": self.min_ped_distance,
               "estops": ag.reflex.events["estop"]}
        if ag.path is not None:
            res.update(path_length=ag.path.length, gear_switches=ag.path.n_gear_switches)
        res["slot"] = ag.target.sid if ag.target else None
        if ag.target is not None and ag.path is not None:
            res["exit_dist"] = ag.selector.exit_distance(ag.target, self.sc.exit_point)
            res["cycle_length"] = ag.path.length + res["exit_dist"]
        lat, lon, dyaw, inside = self.final_pose_error()
        res.update(lat_err=lat, lon_err=lon, yaw_err_deg=dyaw, inside_slot=inside)
        cp = self.p.control
        res["success"] = bool(ag.state == "PARKED" and not self.collided and inside
                              and abs(lat) <= cp.goal_pos_tol and abs(lon) <= cp.goal_pos_tol + 0.2
                              and abs(dyaw) <= math.degrees(cp.goal_yaw_tol))
        if self.log:
            cte = np.array([r["cte"] for r in self.log if r["state"] in ("DRIVE", "DOCK")])
            cte = cte[np.isfinite(cte)]
            res["mean_abs_cte"] = float(np.mean(np.abs(cte))) if len(cte) else float("nan")
            res["max_abs_cte"] = float(np.max(np.abs(cte))) if len(cte) else float("nan")
        return res

    def final_pose_error(self):
        """Lateral / longitudinal / heading error of the car body w.r.t. the slot."""
        ag = self.agent
        if ag.target is None:
            return math.nan, math.nan, math.nan, False
        s = self.sc.slot(ag.target.sid)
        vp, lp = self.p.vehicle, self.p.lot
        fp = car_footprint(self.car.x, self.car.y, self.car.yaw, vp)
        # Lateral error: body centre vs slot centre line; longitudinal: vs ideal body centre.
        lat = fp.cx - s.cx
        if ag.planner_name.startswith("hybrid"):
            gx, gy, gyaw = s.park_pose(self.p)
        else:
            back = s.cy - s.opening * lp.slot_depth / 2
            gy = back + s.opening * (0.5 + vp.front_overhang + vp.wheelbase)
            gx, gyaw = s.cx, -s.opening * math.pi / 2
        ideal = car_footprint(gx, gy, gyaw, vp)
        lon = fp.cy - ideal.cy
        dyaw = math.degrees(abs(wrap_angle(self.car.yaw - gyaw)))
        corners = fp.corners()
        inside = bool(np.all(np.abs(corners[:, 0] - s.cx) <= lp.slot_width / 2 + 0.10) and
                      np.all(np.abs(corners[:, 1] - s.cy) <= lp.slot_depth / 2 + 0.10))
        return lat, lon, dyaw, inside
