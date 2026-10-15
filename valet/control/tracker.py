"""Procedural layer: executes a planned path segment by segment.

At every control step it
  1. finds the nearest waypoint on the active constant-direction segment,
  2. computes the steering angle with the chosen law (Pure Pursuit / Stanley),
  3. computes a reference speed from a trapezoidal profile (cruise, curvature
     limit, braking to the end of the segment),
  4. closes the speed loop with a PID that outputs an acceleration command,
  5. switches to the next segment once the car has stopped at a cusp.
"""
import math
import numpy as np

from .pid import PID
from .steering import PurePursuit, Stanley, nearest_index, cross_track
from ..geometry import wrap_angle

LAWS = {"pure_pursuit": PurePursuit, "stanley": Stanley}


class PathTracker:
    def __init__(self, p, law="pure_pursuit"):
        self.p = p
        vp, cp = p.vehicle, p.control
        self.law = LAWS[law](vp, cp)
        self.speed_pid = PID(cp.speed_kp, cp.speed_ki, cp.speed_kd, cp.dt,
                             -3.0, vp.max_accel)
        self.path = None

    def set_path(self, path):
        self.path = path
        self.segments = path.segments()
        self.seg_idx = 0
        self.wp = 0
        self.speed_pid.reset()
        self.done = False

    @property
    def segment(self):
        return self.segments[self.seg_idx]

    @property
    def on_last_segment(self):
        return self.seg_idx == len(self.segments) - 1

    def remaining(self, i):
        seg = self.segment
        return float(np.sum(np.hypot(np.diff(seg.x[i:]), np.diff(seg.y[i:]))))

    def reference_speed(self, seg, i, d_end, v=0.0):
        vp, cp = self.p.vehicle, self.p.control
        vmax = cp.cruise_speed if seg.direction[0] > 0 else min(cp.manoeuvre_speed, vp.max_reverse_speed)
        # curvature limit (lateral acceleration <= 1.0 m/s^2) over the next 3 m
        k = seg.curvature()[i:i + 12]
        kmax = float(k.max()) if len(k) else 0.0
        if kmax > 1e-3:
            vmax = min(vmax, max(cp.manoeuvre_speed, math.sqrt(1.0 / kmax)))
        # braking to stop at the end of the segment: decel 0.6 m/s^2, and the
        # distance covered during the drivetrain lag (~|v| * 0.5 s) is subtracted
        d_brake = max(d_end - 0.03 - 0.5 * abs(v), 0.0)
        vmax = min(vmax, math.sqrt(2 * 0.6 * d_brake) + 0.10)
        return vmax

    def step(self, x, y, yaw, v):
        """Returns (steer, signed v_ref, info). The speed loop is closed later
        by ``speed_command`` so the reflexive layer can override v_ref first."""
        seg = self.segment
        i = nearest_index(seg, x, y, self.wp, 40)
        self.wp = i
        d_end = self.remaining(i)
        # project past the last waypoint -> segment finished
        ex, ey = seg.x[-1], seg.y[-1]
        dirx = seg.direction[-1] * math.cos(seg.yaw[-1])
        diry = seg.direction[-1] * math.sin(seg.yaw[-1])
        past_end = (x - ex) * dirx + (y - ey) * diry > -0.05
        finished = (d_end < 0.12 or past_end) and i >= len(seg) - 3
        if finished and abs(v) < 0.08:
            if self.on_last_segment:
                self.done = True
                return 0.0, 0.0, {"cte": 0.0, "yaw_err": 0.0, "seg": self.seg_idx, "d_end": 0.0}
            self.seg_idx += 1
            self.wp = 0
            self.speed_pid.reset()
            return self.step(x, y, yaw, v)

        steer = self.law(seg, i, x, y, yaw, v)
        vmax = self.p.vehicle.max_steer
        steer = max(-vmax, min(vmax, steer))
        v_ref = 0.0 if finished else self.reference_speed(seg, i, d_end, v)
        v_ref_signed = seg.direction[0] * v_ref
        cte = cross_track(seg, i, x, y)
        yaw_err = wrap_angle(seg.yaw[i] - yaw)
        return steer, v_ref_signed, {"cte": cte, "yaw_err": yaw_err, "seg": self.seg_idx,
                                      "d_end": d_end}

    def speed_command(self, v_ref, v):
        """PID speed loop -> acceleration command [m/s^2]."""
        return self.speed_pid(v_ref, v)
