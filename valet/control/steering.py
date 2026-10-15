"""Path-tracking steering laws for forward and reverse driving.

Both laws work on the rear-axle reference point and support reverse
driving by expressing every angle relative to the direction of motion
(psi = yaw for forward, yaw + pi for reverse) and flipping the sign of the
steering command when reversing, because d(yaw)/dt = v tan(delta) / L with v < 0.
"""
import math
import numpy as np

from ..geometry import wrap_angle


def nearest_index(path, x, y, start=0, window=60):
    lo, hi = start, min(start + window, len(path))
    d = np.hypot(path.x[lo:hi] - x, path.y[lo:hi] - y)
    return lo + int(np.argmin(d))


def cross_track(path, i, x, y):
    """Signed lateral error of (x, y) w.r.t. the path tangent at i (left of path = +)."""
    tx, ty = path.x[min(i + 1, len(path) - 1)] - path.x[max(i - 1, 0)], \
        path.y[min(i + 1, len(path) - 1)] - path.y[max(i - 1, 0)]
    n = math.hypot(tx, ty) or 1.0
    return (tx * (y - path.y[i]) - ty * (x - path.x[i])) / n


class PurePursuit:
    name = "pure_pursuit"

    def __init__(self, vp, cp):
        self.L, self.vp, self.cp = vp.wheelbase, vp, cp

    def target_point(self, path, i, ld):
        """Point at arc length ld ahead of index i (extended past the end)."""
        s = 0.0
        for k in range(i, len(path) - 1):
            seg = math.hypot(path.x[k + 1] - path.x[k], path.y[k + 1] - path.y[k])
            if s + seg >= ld:
                f = (ld - s) / seg
                return (path.x[k] + f * (path.x[k + 1] - path.x[k]),
                        path.y[k] + f * (path.y[k + 1] - path.y[k]))
            s += seg
        # extend along the final heading (in the direction of motion)
        d = path.direction[-1]
        rem = ld - s
        return (path.x[-1] + d * rem * math.cos(path.yaw[-1]),
                path.y[-1] + d * rem * math.sin(path.yaw[-1]))

    def __call__(self, path, i, x, y, yaw, v):
        d = path.direction[i]
        ld = max(self.cp.lookahead_min, self.cp.lookahead_gain * abs(v))
        if d < 0:
            ld = max(1.2, 0.8 * ld)
        tx, ty = self.target_point(path, i, ld)
        psi = yaw if d > 0 else yaw + math.pi
        alpha = wrap_angle(math.atan2(ty - y, tx - x) - psi)
        kappa = 2.0 * math.sin(alpha) / ld
        delta = math.atan(self.L * kappa)
        return d * delta


class Stanley:
    """Stanley controller (Thrun et al., 2006; Hoffmann et al., 2007).

    The planner returns a rear-axle path, while Stanley is defined for the
    front axle. Forward: the path is shifted to the front axle
    (x + L cos yaw, y + L sin yaw) before tracking. Reverse: the rear axle
    leads, so it is used as the reference and a curvature feed-forward
    delta_ff = atan(L * kappa) is added (without it the car needs a large
    steady-state error to hold an arc).
    """
    name = "stanley"

    def __init__(self, vp, cp):
        self.L, self.vp, self.cp = vp.wheelbase, vp, cp
        self._cache = {}

    def _front_path(self, path):
        key = id(path)
        if key not in self._cache:
            from ..planning.path import Path
            fx, fy = path.x + self.L * np.cos(path.yaw), path.y + self.L * np.sin(path.yaw)
            # heading of the front-axle path = its own tangent (= yaw + delta on arcs)
            tang = np.arctan2(np.gradient(fy), np.gradient(fx)) if len(fx) > 1 else path.yaw
            self._cache = {key: Path(fx, fy, tang, path.direction)}
        return self._cache[key]

    def __call__(self, path, i, x, y, yaw, v):
        d = path.direction[i]
        k, ks = self.cp.stanley_k, self.cp.stanley_soft
        if d > 0:
            fp = self._front_path(path)
            fx, fy = x + self.L * math.cos(yaw), y + self.L * math.sin(yaw)
            j = nearest_index(fp, fx, fy, max(i - 5, 0), 80)
            e = cross_track(fp, j, fx, fy)                # + when the axle is left of the path
            psi_e = wrap_angle(fp.yaw[j] - yaw)
            return psi_e + math.atan2(-k * e, ks + abs(v))
        # Reverse: path points are ordered along the motion, so e is measured
        # w.r.t. the direction of motion (+ = car left of the path).
        e = cross_track(path, i, x, y)
        psi_e = wrap_angle(path.yaw[i] - yaw)
        j2 = min(i + 2, len(path) - 1)
        ds = math.hypot(path.x[j2] - path.x[i], path.y[j2] - path.y[i])
        kappa = wrap_angle(path.yaw[j2] - path.yaw[i]) / ds if ds > 1e-6 else 0.0
        # d(yaw)/ds_motion = -tan(delta)/L when reversing -> delta_ff = -atan(L * kappa_motion)
        # and kappa_motion = d(yaw)/ds (yaw and motion heading differ by pi).
        delta_ff = -math.atan(self.L * kappa)
        return delta_ff - (psi_e + math.atan2(-k * e, ks + abs(v)))
