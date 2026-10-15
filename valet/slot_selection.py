"""Declarative layer, part 1: which free slot should the valet agent use?

Two policies are compared in the experiments:

* ``nearest``  - the free slot whose entry point is closest (straight line)
                 to the car: what a simple rule-based system does.
* ``utility``  - a utility-based agent (Russell & Norvig, ch. 2) that
                 minimises an estimated cost

        J(s) = D_route(s) + w_exit * D_exit(s) + w_man * M(s)

  D_route : obstacle-aware driving distance from the car to the slot
            (one Dijkstra run on the inflated map covers every slot),
  D_exit  : driving distance from the slot to the pick-up zone
            (the valet must also retrieve the car later),
  M(s)    : manoeuvre difficulty = occupied neighbours (0-2)
            + occupied slot opposite the mouth (less room to swing)
            + 1 if the slot is at the end wall (no room to pull past).
"""
import math
import numpy as np

from .planning.hybrid_astar import dijkstra_heuristic


def _neighbours(slot, slots):
    same_row = {s.index: s for s in slots if s.row == slot.row}
    return [same_row[k] for k in (slot.index - 1, slot.index + 1) if k in same_row]


def _opposite(slot, slots):
    pair = {"A": "B", "B": "A", "C": "D", "D": "C"}[slot.row]
    return next(s for s in slots if s.row == pair and s.index == slot.index)


def difficulty(slot, slots, n_per_row):
    m = sum(1 for n in _neighbours(slot, slots) if n.occupied)
    m += 1 if _opposite(slot, slots).occupied else 0
    m += 1 if slot.index == n_per_row - 1 else 0
    return m


class SlotSelector:
    def __init__(self, costmap, policy="utility", w_exit=1.0, w_man=4.0, res=0.5):
        self.cm, self.policy = costmap, policy
        self.w_exit, self.w_man, self.res = w_exit, w_man, res
        vp = costmap.p.vehicle
        self.blocked = costmap.inflated_grid(res, vp.width / 2 - 0.1)

    def _dist_map(self, x, y):
        i, j = int(y / self.res), int(x / self.res)
        b = self.blocked.copy()
        b[max(i - 2, 0):i + 3, max(j - 2, 0):j + 3] = False
        return dijkstra_heuristic(b, (i, j), self.res)

    def _lookup(self, dmap, x, y):
        i, j = int(y / self.res), int(x / self.res)
        v = dmap[i, j]
        return float(v) if np.isfinite(v) else 1e3

    def exit_distance(self, slot, exit_point):
        """Driving distance from the slot mouth to the pick-up zone [m]."""
        ex, ey, _ = slot.entry_pose(self.cm.p)
        return self._lookup(self._dist_map(*exit_point), ex, ey)

    def rank(self, pose, slots, exit_point, exclude=()):
        free = [s for s in slots if not s.occupied and s.sid not in exclude]
        if not free:
            return []
        p = self.cm.p
        if self.policy == "nearest":
            scored = [(math.hypot(s.entry_pose(p)[0] - pose[0], s.entry_pose(p)[1] - pose[1]), s)
                      for s in free]
        else:
            d_car = self._dist_map(pose[0], pose[1])
            d_exit = self._dist_map(*exit_point)
            n = p.lot.slots_per_row
            scored = []
            for s in free:
                ex, ey, _ = s.entry_pose(p)
                j = self._lookup(d_car, ex, ey) + self.w_exit * self._lookup(d_exit, ex, ey) \
                    + self.w_man * difficulty(s, slots, n)
                scored.append((j, s))
        scored.sort(key=lambda t: t[0])
        return [s for _, s in scored]
