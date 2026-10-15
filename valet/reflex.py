"""Reflexive layer: fast sensor-to-actuator rules that can override the
procedural layer (subsumption-style, Brooks 1986). Highest priority first:

  R1  EMERGENCY STOP   the first sensed point that the car body would touch
                       along its current steering arc is closer than d_stop.
  R2  SPEED LIMIT      mode "ttc"  (ours):    v <= (s_c - d_stop) / T_safe - v_obstacle
                       mode "fixed" (baseline): v = 0 if a straight corridor
                       ahead is blocked closer than d_fixed.
  R3  DOCKING          in the final reverse into the slot: centre between the
                       neighbouring cars using the side sensors, and stop hard
                       if the rear gap is below dock_rear_gap.
  R4  BLOCKED          held at zero speed for blocked_replan_time -> flag the
                       declarative layer to re-plan around the obstacle.

Swept-footprint contact distance s_c (project novelty): every range reading
is turned into a point in the car frame; the car body (inflated by a margin)
is then moved along the arc given by the CURRENT steering angle, and s_c is
the arc length at which the first point enters the body. Unlike a straight
"corridor ahead of the bumper" this also catches what the side of the car
would sweep while turning - which is how most parking-lot scrapes happen -
and it ignores parked cars that are close but not on the swept path.
"""
import math
import numpy as np


class ReflexLayer:
    FRONT = ("front_center", "front_left_in", "front_right_in", "front_left", "front_right")
    REAR = ("rear_center", "rear_left_in", "rear_right_in", "rear_left", "rear_right")
    DOCK_LEFT = tuple(f"side_left_{k}" for k in (1, 2, 3, 4))
    DOCK_RIGHT = tuple(f"side_right_{k}" for k in (1, 2, 3, 4))

    def __init__(self, p, mode="ttc"):
        assert mode in ("none", "fixed", "ttc")
        self.p, self.mode = p, mode
        self.mounts = {m[0]: m[1:] for m in p.sensors.mounts}
        self.prev_d = None
        self.blocked_time = 0.0
        self.events = {"estop": 0, "limited_steps": 0, "dock_stop": 0}
        self._s = np.arange(0.0, 6.01, 0.1)
        self.contact_point = None

    # --------------------------------------------------------------- geometry
    def hit_points(self, ranges, names=None):
        mr = self.p.sensors.max_range
        pts = []
        for name, (mx, my, a) in self.mounts.items():
            if names is not None and name not in names:
                continue
            r = ranges.get(name, mr)
            if r < mr - 1e-6:
                pts.append((mx + r * math.cos(a), my + r * math.sin(a)))
        return np.array(pts) if pts else np.zeros((0, 2))

    def corridor_distance(self, ranges, direction):
        """Baseline: straight corridor (half width + 0.3 m) ahead of the bumper."""
        vp = self.p.vehicle
        half = vp.width / 2 + 0.3
        bumper = vp.wheelbase + vp.front_overhang if direction > 0 else -vp.rear_overhang
        pts = self.hit_points(ranges, self.FRONT if direction > 0 else self.REAR)
        if not len(pts):
            return math.inf
        ok = np.abs(pts[:, 1]) <= half
        if not ok.any():
            return math.inf
        return float(max(np.min(direction * (pts[ok, 0] - bumper)), 0.0))

    def swept_contact_distance(self, ranges, direction, steer, horizon):
        """Arc length until the inflated body first touches a sensed point.
        Also stores that point (car frame) in self.contact_point for R4."""
        vp, m = self.p.vehicle, self.p.reflex.swept_margin
        self.contact_point = None
        pts = self.hit_points(ranges)
        if not len(pts):
            return math.inf
        s = self._s[self._s <= horizon]
        k = math.tan(steer) / vp.wheelbase
        th = direction * s * k
        if abs(k) < 1e-4:
            px, py = direction * s, np.zeros_like(s)
        else:
            px, py = np.sin(th) / k, (1 - np.cos(th)) / k
        c, sn = np.cos(th)[:, None], np.sin(th)[:, None]
        dx, dy = pts[None, :, 0] - px[:, None], pts[None, :, 1] - py[:, None]
        lx, ly = c * dx + sn * dy, -sn * dx + c * dy
        inside = ((lx >= -vp.rear_overhang - m) & (lx <= vp.wheelbase + vp.front_overhang + m)
                  & (np.abs(ly) <= vp.width / 2 + m))
        hit_rows = np.nonzero(inside.any(axis=1))[0]
        if not len(hit_rows):
            return math.inf
        k0 = hit_rows[0]
        self.contact_point = tuple(pts[np.nonzero(inside[k0])[0][0]])
        return float(s[k0])

    # ------------------------------------------------------------------ apply
    def apply(self, steer, v_ref, v, ranges, dt, docking=False):
        """Returns (steer, v_ref, info)."""
        rp = self.p.reflex
        info = {"rule": "-", "d": math.inf, "dock_stop": False, "blocked": False}
        if self.mode == "none" or (abs(v_ref) < 1e-6 and not docking):
            self.blocked_time = 0.0
            return steer, v_ref, info
        direction = 1 if v_ref > 0 else -1

        if docking:
            # R3: rear gap and lateral centring (only when both neighbours are seen)
            rear = ranges.get("rear_center", math.inf)
            if rear < rp.dock_rear_gap:
                info.update(rule="R3-stop", dock_stop=True)
                self.events["dock_stop"] += 1
                return steer, 0.0, info
            dl = min(ranges.get(n, math.inf) for n in self.DOCK_LEFT)
            dr = min(ranges.get(n, math.inf) for n in self.DOCK_RIGHT)
            if dl < 1.5 and dr < 1.5:
                steer = steer + float(np.clip(rp.centring_gain * (dl - dr), -0.08, 0.08))
                info["rule"] = "R3-centre"
            return steer, v_ref, info

        if self.mode == "ttc":
            horizon = min(6.0, abs(v) * rp.ttc_safe + rp.stop_distance + 1.0)
            d = self.swept_contact_distance(ranges, direction, steer, horizon)
        else:
            d = self.corridor_distance(ranges, direction)
            self.contact_point = None
        info["d"] = d
        info["contact"] = self.contact_point
        # closing speed of the obstacle itself (range rate minus own speed)
        v_obs = 0.0
        if self.prev_d is not None and math.isfinite(d):
            v_obs = min(max(0.0, (self.prev_d - d) / dt - abs(v)), 2.0)
        self.prev_d = d if math.isfinite(d) else None

        limited = abs(v_ref)
        if d < rp.stop_distance:
            limited = 0.0
            info["rule"] = "R1-estop"
            self.events["estop"] += 1
        elif self.mode == "ttc":
            v_allow = max(0.0, (d - rp.stop_distance) / rp.ttc_safe - v_obs)
            if v_allow < limited:
                limited = max(v_allow, 0.0)
                info["rule"] = "R2-ttc"
        elif self.mode == "fixed" and d < rp.fixed_slow_distance:
            limited = 0.0
            info["rule"] = "R2-fixed"
        if limited < abs(v_ref) - 1e-6:
            self.events["limited_steps"] += 1
        # R4: held at (almost) zero speed by the reflexes
        if limited < 0.05 and abs(v_ref) > 0.2:
            self.blocked_time += dt
        else:
            self.blocked_time = 0.0
        info["blocked"] = self.blocked_time > rp.blocked_replan_time
        return steer, direction * limited, info
