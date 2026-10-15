"""Deliberative planner: Hybrid A* with a slot "goal funnel".

Search space: continuous pose (x, y, yaw) with a discrete closed set on
(x, y, yaw) cells (Dolgov et al., 2010). Each expansion applies Ackermann
motion primitives (n_steer steering angles x forward/reverse) integrated with
the kinematic bicycle model, so every path the planner returns is drivable by
the car.

Cost  g = sum(arc length x [reverse_cost if reversing])
        + steer_cost * |delta| + steer_change_cost * |delta - delta_prev|
        + gear_switch_cost * [direction changed]
Heuristic h = max(Euclidean distance, 2-D Dijkstra distance on the inflated
map from the goal), which accounts for obstacles such as parked-car rows.

Goal funnel (project novelty): instead of hitting the exact parked pose,
the search may stop at any pose that lies on the slot axis outside the slot,
within a small lateral and heading tolerance; the remaining motion is a
collision-checked straight reverse into the slot. This turns a tiny goal
region into a long, thin one and cuts node expansions about 5x (E1), while
the procedural/reflexive layers remove the residual offset.
"""
import heapq
import math
import time
import numpy as np

from .path import Path
from ..geometry import wrap_angle


class _Node:
    __slots__ = ("x", "y", "yaw", "g", "parent", "direction", "steer", "traj")

    def __init__(self, x, y, yaw, g, parent, direction, steer, traj):
        self.x, self.y, self.yaw, self.g = x, y, yaw, g
        self.parent, self.direction, self.steer, self.traj = parent, direction, steer, traj


def dijkstra_heuristic(blocked, goal_cell, res):
    """2-D Dijkstra (8-connected) from the goal cell; returns a distance grid [m]."""
    ny, nx = blocked.shape
    dist = np.full((ny, nx), np.inf)
    dist[goal_cell] = 0.0
    q = [(0.0, goal_cell)]
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    while q:
        d, (i, j) = heapq.heappop(q)
        if d > dist[i, j]:
            continue
        for di, dj, c in moves:
            ni, nj = i + di, j + dj
            if 0 <= ni < ny and 0 <= nj < nx and not blocked[ni, nj]:
                nd = d + c * res
                if nd < dist[ni, nj]:
                    dist[ni, nj] = nd
                    heapq.heappush(q, (nd, (ni, nj)))
    return dist


class HybridAStarPlanner:
    def __init__(self, costmap, use_funnel=True, record_tree=False):
        self.cm = costmap
        self.record_tree = record_tree
        self.tree = []                      # (x0, y0, x1, y1) of expanded edges, for figures
        self.p = costmap.p
        self.use_funnel = use_funnel
        pp, vp = self.p.planner, self.p.vehicle
        self.res = pp.xy_resolution
        self.yaw_res = 2 * math.pi / pp.yaw_bins
        smax = pp.steer_fraction * vp.max_steer
        steers = np.linspace(-smax, smax, pp.n_steer)
        self.primitives = [(d, s) for d in (1, -1) for s in steers]
        # Heuristic map: inflate by half the width only, so narrow-but-passable
        # gaps stay open and the heuristic stays optimistic.
        self.h_blocked = costmap.inflated_grid(self.res, vp.width / 2 - 0.1)

    # ------------------------------------------------------------------ helpers
    def _key(self, x, y, yaw):
        return (int(x / self.res), int(y / self.res), int(wrap_angle(yaw) // self.yaw_res))

    def _simulate(self, x, y, yaw, direction, steer):
        """Integrate the bicycle model over one primitive (sub-steps of 0.25 m)."""
        pp, L = self.p.planner, self.p.vehicle.wheelbase
        n = int(math.ceil(pp.step / 0.25))
        ds = direction * pp.step / n
        xs, ys, yaws = np.empty(n), np.empty(n), np.empty(n)
        k = math.tan(steer) / L
        for i in range(n):
            x += ds * math.cos(yaw)
            y += ds * math.sin(yaw)
            yaw = wrap_angle(yaw + ds * k)
            xs[i], ys[i], yaws[i] = x, y, yaw
        return xs, ys, yaws

    def _h(self, x, y):
        i, j = int(y / self.res), int(x / self.res)
        ny, nx = self.hmap.shape
        hd = self.hmap[i, j] if (0 <= i < ny and 0 <= j < nx) else np.inf
        if not np.isfinite(hd):
            hd = 1e3
        return max(hd, math.hypot(x - self.goal[0], y - self.goal[1]))

    def _funnel(self, node):
        """Try to finish with a straight reverse into the slot (goal funnel)."""
        gx, gy, gyaw = self.goal
        pp = self.p.planner
        c, s = math.cos(gyaw), math.sin(gyaw)
        dx, dy = node.x - gx, node.y - gy
        along, lateral = c * dx + s * dy, -s * dx + c * dy
        dyaw = wrap_angle(node.yaw - gyaw)
        tol_xy = pp.funnel_lateral_tol if self.use_funnel else 0.15
        tol_yaw = pp.funnel_heading_tol if self.use_funnel else math.radians(3)
        max_d = pp.funnel_max_dist if self.use_funnel else 0.3
        min_d = pp.funnel_min_dist if self.use_funnel else -0.05
        if not (abs(lateral) < tol_xy and abs(dyaw) < tol_yaw and min_d <= along <= max_d):
            return None
        n = max(int(along / 0.25), 1)
        t = np.linspace(0.0, 1.0, n + 1)[1:]
        xs, ys = node.x + t * (gx - node.x), node.y + t * (gy - node.y)
        yaws = wrap_angle(node.yaw - t * dyaw)
        if not self.cm.poses_free(xs, ys, yaws, margin=0.05):
            return None
        return xs, ys, yaws

    # --------------------------------------------------------------------- plan
    def plan(self, start, goal, max_expansions=None):
        t0 = time.perf_counter()
        pp = self.p.planner
        budget = max_expansions or pp.max_expansions
        self.goal = goal
        gi, gj = int(goal[1] / self.res), int(goal[0] / self.res)
        hb = self.h_blocked.copy()
        hb[max(gi - 2, 0):gi + 3, max(gj - 2, 0):gj + 3] = False
        self.hmap = dijkstra_heuristic(hb, (gi, gj), self.res)

        root = _Node(*start, 0.0, None, 1, 0.0, (np.array([start[0]]), np.array([start[1]]),
                                                  np.array([start[2]])))
        openq = [(self._h(start[0], start[1]), 0, root)]
        best_g = {self._key(*start): 0.0}
        closed = set()
        counter = 1
        expansions = 0
        while openq and expansions < budget:
            _, _, node = heapq.heappop(openq)
            key = self._key(node.x, node.y, node.yaw)
            if key in closed:
                continue
            closed.add(key)
            expansions += 1
            tail = self._funnel(node)
            if tail is not None:
                path = self._extract(node, tail)
                path.info = {"time": time.perf_counter() - t0, "expansions": expansions,
                             "ok": True, "planner": "hybrid_astar" + ("" if self.use_funnel else "_nofunnel")}
                return path, path.info
            for direction, steer in self.primitives:
                xs, ys, yaws = self._simulate(node.x, node.y, node.yaw, direction, steer)
                nkey = self._key(xs[-1], ys[-1], yaws[-1])
                if nkey in closed:
                    continue
                if not self.cm.poses_free(xs, ys, yaws):
                    continue
                cost = pp.step * (pp.reverse_cost if direction < 0 else 1.0)
                cost += pp.steer_cost * abs(steer) + pp.steer_change_cost * abs(steer - node.steer)
                if direction != node.direction:
                    cost += pp.gear_switch_cost
                g = node.g + cost
                if g >= best_g.get(nkey, math.inf):
                    continue
                best_g[nkey] = g
                if self.record_tree:
                    self.tree.append((node.x, node.y, xs[-1], ys[-1]))
                child = _Node(xs[-1], ys[-1], yaws[-1], g, node, direction, steer, (xs, ys, yaws))
                heapq.heappush(openq, (g + self._h(xs[-1], ys[-1]), counter, child))
                counter += 1
        return None, {"time": time.perf_counter() - t0, "expansions": expansions, "ok": False,
                      "planner": "hybrid_astar"}

    def _extract(self, node, tail):
        trajs, dirs = [], []
        while node is not None:
            trajs.append(node.traj)
            dirs.append(np.full(len(node.traj[0]), node.direction))
            node = node.parent
        trajs, dirs = trajs[::-1], dirs[::-1]
        x = np.concatenate([t[0] for t in trajs] + [tail[0]])
        y = np.concatenate([t[1] for t in trajs] + [tail[1]])
        yaw = np.concatenate([t[2] for t in trajs] + [tail[2]])
        d = np.concatenate(dirs + [np.full(len(tail[0]), -1)])
        if len(d) > 1:          # the root carries no motion
            d[0] = d[1]
        return Path(x, y, yaw, d)
