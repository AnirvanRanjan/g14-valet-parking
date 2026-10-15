"""Baseline deliberative planner: 8-connected grid A* on an inflated map.

The car is treated as a disc (obstacles are inflated by half the car width
plus a margin), so the path ignores heading and the Ackermann turning-radius
constraint. This is the textbook planner from the syllabus (Hart, Nilsson and
Raphael, 1968) and is the baseline that Hybrid A* is compared against.
"""
import heapq
import math
import time
import numpy as np

from .path import from_points

SQRT2 = math.sqrt(2.0)
MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


def astar_grid(blocked, start, goal):
    """A* on a boolean grid. start/goal are (i, j). Returns cell list and #expansions."""
    ny, nx = blocked.shape
    gi, gj = goal

    def h(i, j):  # octile distance: admissible and consistent for 8-connectivity
        di, dj = abs(i - gi), abs(j - gj)
        return (di + dj) + (SQRT2 - 2) * min(di, dj)

    g = {start: 0.0}
    parent = {start: None}
    openq = [(h(*start), 0.0, start)]
    closed = set()
    expansions = 0
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        closed.add(cur)
        expansions += 1
        if cur == goal:
            cells = []
            while cur is not None:
                cells.append(cur)
                cur = parent[cur]
            return cells[::-1], expansions
        ci, cj = cur
        for di, dj, c in MOVES:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < ny and 0 <= nj < nx) or blocked[ni, nj]:
                continue
            if di and dj and (blocked[ci + di, cj] or blocked[ci, cj + dj]):
                continue          # no corner cutting
            ng = gc + c
            if ng < g.get((ni, nj), math.inf):
                g[(ni, nj)] = ng
                parent[(ni, nj)] = cur
                heapq.heappush(openq, (ng + h(ni, nj), ng, (ni, nj)))
    return None, expansions


def _line_free(blocked, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
    ii = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    jj = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return not blocked[ii, jj].any()


def shortcut(blocked, cells):
    """Greedy line-of-sight smoothing of the grid path."""
    out = [cells[0]]
    k = 0
    while k < len(cells) - 1:
        m = len(cells) - 1
        while m > k + 1 and not _line_free(blocked, cells[k], cells[m]):
            m -= 1
        out.append(cells[m])
        k = m
    return out


class GridAStarPlanner:
    """Plans a forward-only, nose-in route for the rear-axle reference point."""

    def __init__(self, costmap, resolution=0.25):
        self.cm = costmap
        self.res = resolution
        vp, pp = costmap.p.vehicle, costmap.p.planner
        self.blocked = costmap.inflated_grid(resolution, vp.width / 2 + pp.safety_margin)

    def _cell(self, x, y):
        return (int(y / self.res), int(x / self.res))

    def plan(self, start, slot):
        """start = (x, y, yaw). Goal: nose-in pose inside the slot."""
        t0 = time.perf_counter()
        p = self.cm.p
        lp, vp = p.lot, p.vehicle
        back = slot.cy - slot.opening * lp.slot_depth / 2
        gy = back + slot.opening * (0.5 + vp.front_overhang + vp.wheelbase)
        goal_xy = (slot.cx, gy)
        s, g = self._cell(*start[:2]), self._cell(*goal_xy)
        blocked = self.blocked.copy()
        blocked[s] = blocked[g] = False
        cells, n_exp = astar_grid(blocked, s, g)
        dt = time.perf_counter() - t0
        if cells is None:
            return None, {"time": dt, "expansions": n_exp, "ok": False}
        cells = shortcut(blocked, cells)
        pts = [((j + 0.5) * self.res, (i + 0.5) * self.res) for i, j in cells]
        pts[0], pts[-1] = start[:2], goal_xy
        path = from_points(pts, direction=1)
        path.info = {"time": time.perf_counter() - t0, "expansions": n_exp, "ok": True,
                     "planner": "grid_astar"}
        return path, path.info
