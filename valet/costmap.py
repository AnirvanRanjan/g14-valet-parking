"""Occupancy grid + Euclidean distance transform for fast footprint checks.

The car body outline is sampled every ``spacing`` metres (perimeter
points). A pose is collision-free when every perimeter point is farther than
the safety margin from the nearest occupied cell. Because the distance
transform is precomputed, each check is a handful of array lookups, which is
what keeps Hybrid A* fast in Python. (A disc decomposition was tried first,
but its end discs stick out ~0.7 m beyond the bumpers and wrongly reject the
final parked pose next to the back wall.)
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt

from .config import DEFAULT, Params


class CostMap:
    def __init__(self, obstacles, p: Params = DEFAULT, resolution: float = 0.1, spacing: float = 0.2):
        self.p = p
        self.res = resolution
        lp = p.lot
        self.w, self.h = lp.width, lp.height
        self.nx, self.ny = int(math.ceil(self.w / resolution)), int(math.ceil(self.h / resolution))
        self.occ = np.zeros((self.ny, self.nx), dtype=bool)
        for box in obstacles:
            self._rasterise(box)
        # Lot boundary is always an obstacle.
        self.occ[0, :] = self.occ[-1, :] = True
        self.occ[:, 0] = self.occ[:, -1] = True
        self.dist = distance_transform_edt(~self.occ) * resolution

        vp = p.vehicle
        xr, xf, hw = -vp.rear_overhang, vp.wheelbase + vp.front_overhang, vp.width / 2
        nl, nw = int(math.ceil(vp.length / spacing)), int(math.ceil(vp.width / spacing))
        lx = np.linspace(xr, xf, nl + 1)
        ly = np.linspace(-hw, hw, nw + 1)
        pts = np.vstack([np.c_[lx, np.full_like(lx, hw)], np.c_[lx, np.full_like(lx, -hw)],
                         np.c_[np.full_like(ly, xr), ly], np.c_[np.full_like(ly, xf), ly]])
        self.outline = pts                                    # (M, 2) in the rear-axle frame

    def _rasterise(self, box):
        corners = box.corners()
        x0, y0 = corners.min(axis=0)
        x1, y1 = corners.max(axis=0)
        i0, i1 = max(int(y0 / self.res), 0), min(int(y1 / self.res) + 1, self.ny)
        j0, j1 = max(int(x0 / self.res), 0), min(int(x1 / self.res) + 1, self.nx)
        if i0 >= i1 or j0 >= j1:
            return
        ys = (np.arange(i0, i1) + 0.5) * self.res
        xs = (np.arange(j0, j1) + 0.5) * self.res
        X, Y = np.meshgrid(xs, ys)
        c, s = math.cos(box.yaw), math.sin(box.yaw)
        lx = c * (X - box.cx) + s * (Y - box.cy)
        ly = -s * (X - box.cx) + c * (Y - box.cy)
        inside = (np.abs(lx) <= box.length / 2) & (np.abs(ly) <= box.width / 2)
        self.occ[i0:i1, j0:j1] |= inside

    def distance_at(self, x, y):
        """Distance to the nearest obstacle (vectorised, out of map -> 0)."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        j = np.floor(x / self.res).astype(int)
        i = np.floor(y / self.res).astype(int)
        inside = (i >= 0) & (i < self.ny) & (j >= 0) & (j < self.nx)
        d = np.zeros_like(x)
        d[inside] = self.dist[i[inside], j[inside]]
        return d

    def pose_clearance(self, x, y, yaw):
        """Smallest obstacle distance over the body outline; arrays allowed."""
        x, y, yaw = np.atleast_1d(x), np.atleast_1d(y), np.atleast_1d(yaw)
        c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        px = x[:, None] + c * self.outline[None, :, 0] - s * self.outline[None, :, 1]
        py = y[:, None] + s * self.outline[None, :, 0] + c * self.outline[None, :, 1]
        d = self.distance_at(px, py) - 0.5 * self.res * math.sqrt(2)
        return d.min(axis=1)

    def poses_free(self, x, y, yaw, margin=None):
        margin = self.p.planner.safety_margin if margin is None else margin
        return bool(np.all(self.pose_clearance(x, y, yaw) > margin))

    def inflated_grid(self, resolution: float, inflation: float):
        """Coarse boolean grid (True = blocked) for 2-D A* / heuristic search."""
        nx, ny = int(math.ceil(self.w / resolution)), int(math.ceil(self.h / resolution))
        xs = (np.arange(nx) + 0.5) * resolution
        ys = (np.arange(ny) + 0.5) * resolution
        X, Y = np.meshgrid(xs, ys)
        return self.distance_at(X, Y) <= inflation
