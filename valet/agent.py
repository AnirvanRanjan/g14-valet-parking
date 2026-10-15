"""The valet parking agent: a three-layer (declarative - procedural -
reflexive) hybrid architecture, driven by a mission state machine.

    +-------------------------------------------------------------+
    | DECLARATIVE  slot selection (utility)  +  route planning    |  ~ once per mission
    |              (Hybrid A* with goal funnel / grid A*)         |    (or on events)
    +-------------------------------------------------------------+
    | PROCEDURAL   path tracking (Pure Pursuit / Stanley),        |  every 50 ms
    |              speed profile + PID, segment / cusp sequencing |
    +-------------------------------------------------------------+
    | REFLEXIVE    E-stop, TTC speed limit, docking rules,        |  every 50 ms,
    |              blocked detection                              |  overrides above
    +-------------------------------------------------------------+

Mission FSM:  SELECT -> PLAN -> DRIVE -> DOCK -> PARKED
                 ^        |       |  \
                 +--------+-------+   +--> PLAN (blocked: re-plan)
              (plan failed / slot taken by another car)
"""
from dataclasses import dataclass, field
import math
import time

from .config import DEFAULT
from .costmap import CostMap
from .geometry import OBB
from .planning import GridAStarPlanner, HybridAStarPlanner
from .control import PathTracker
from .reflex import ReflexLayer
from .localisation import PoseFilter
from .slot_selection import SlotSelector


@dataclass
class Command:
    steer: float
    accel: float
    v_ref: float
    state: str
    info: dict = field(default_factory=dict)


class ValetAgent:
    def __init__(self, slots, static_boxes, exit_point, p=DEFAULT, planner="hybrid",
                 law="pure_pursuit", reflex="ttc", selector="utility", max_slot_tries=4,
                 localisation=True):
        self.p = p
        self.slots = slots                         # shared occupancy view (infrastructure)
        self.static_boxes = list(static_boxes)     # walls + parked cars (from infrastructure)
        self.temp_boxes = []                       # obstacles added after a blocked event
        self.exit_point = exit_point
        self.planner_name, self.selector_name = planner, selector
        self.tracker = PathTracker(p, law)
        self.reflex = ReflexLayer(p, reflex)
        self.max_slot_tries = max_slot_tries
        self.pose_filter = PoseFilter() if localisation else None
        self.state = "SELECT"
        self.target = None
        self.path = None
        self.tried = []
        self.stats = {"plan_time": 0.0, "plans": 0, "expansions": 0, "replans": 0,
                      "reselections": 0, "plan_fail": 0}
        self._rebuild()

    # ------------------------------------------------------------ declarative
    def _rebuild(self):
        self.costmap = CostMap(self.static_boxes + self.temp_boxes, self.p)
        if self.planner_name == "hybrid":
            self.planner = HybridAStarPlanner(self.costmap, use_funnel=True)
        elif self.planner_name == "hybrid_nofunnel":
            self.planner = HybridAStarPlanner(self.costmap, use_funnel=False)
        else:
            self.planner = GridAStarPlanner(self.costmap)
        self.selector = SlotSelector(self.costmap, self.selector_name)

    def _select(self, pose):
        ranked = self.selector.rank(pose, self.slots, self.exit_point, exclude=self.tried)
        if not ranked or len(self.tried) >= self.max_slot_tries:
            self.state = "FAILED"
            return
        self.target = ranked[0]
        self.tried.append(self.target.sid)
        self.state = "PLAN"

    def _plan(self, pose, budget=None):
        t0 = time.perf_counter()
        if self.planner_name.startswith("hybrid"):
            path, info = self.planner.plan(pose, self.target.park_pose(self.p), budget)
        else:
            path, info = self.planner.plan(pose, self.target)
        self.stats["plan_time"] += time.perf_counter() - t0
        self.stats["plans"] += 1
        self.stats["expansions"] += info.get("expansions", 0)
        if path is None:
            self.stats["plan_fail"] += 1
            self.state = "SELECT"
            return
        self.path = path
        self.tracker.set_path(path)
        self.state = "DRIVE"

    def notify_occupied(self, sid, box):
        """Infrastructure message: slot ``sid`` has just been taken by another car."""
        slot = next(s for s in self.slots if s.sid == sid)
        slot.occupied = True
        self.static_boxes.append(box)
        self._rebuild()
        if self.target is not None and self.target.sid == sid and self.state not in ("PARKED", "FAILED"):
            self.stats["reselections"] += 1
            self.state = "SELECT"

    # ----------------------------------------------------------------- step
    def step(self, t, x, y, yaw, v, ranges, dt):
        """x, y: raw GPS; yaw: IMU; v: wheel-speed; ranges: {sensor name: metres}."""
        if self.pose_filter is not None:
            x, y = self.pose_filter.update(x, y, yaw, v, dt)
        pose = (x, y, yaw)
        if self.state == "SELECT":
            self._select(pose)
        if self.state == "PLAN":
            self._plan(pose)
            if self.state == "SELECT":          # planning failed -> next slot now
                self._select(pose)
                if self.state == "PLAN":
                    self._plan(pose)
        if self.state not in ("DRIVE", "DOCK"):
            accel = self.tracker.speed_command(0.0, v)
            return Command(0.0, accel, 0.0, self.state)

        # ---------------------------------------------------------- procedural
        steer, v_ref, tinfo = self.tracker.step(x, y, yaw, v)
        if self.tracker.done:
            self.state = "PARKED"
            return Command(0.0, self.tracker.speed_command(0.0, v), 0.0, self.state, tinfo)
        docking = (self.planner_name.startswith("hybrid") and self.tracker.on_last_segment
                   and self.tracker.segment.direction[0] < 0 and tinfo["d_end"] < 5.0)
        if docking:
            self.state = "DOCK"

        # ----------------------------------------------------------- reflexive
        steer, v_ref, rinfo = self.reflex.apply(steer, v_ref, v, ranges, dt, docking)
        if rinfo["dock_stop"]:
            self.tracker.done = True
        if rinfo["blocked"]:
            self._handle_blocked(pose, rinfo["d"], v_ref, rinfo.get("contact"))
        accel = self.tracker.speed_command(v_ref, v)
        if self.tracker.done and abs(v) < 0.05:
            self.state = "PARKED"
        info = dict(tinfo, **{"rule": rinfo["rule"], "d_obs": rinfo["d"]})
        return Command(steer, accel, v_ref, self.state, info)

    def _handle_blocked(self, pose, d, v_ref, contact=None):
        """R4: the swept path has been blocked for a while -> ask the declarative
        layer for a route around the obstacle. The obstacle is placed where the
        sensors saw it (contact point) or, for the straight-corridor baseline,
        straight ahead. If no route exists, keep the old path and keep waiting."""
        x, y, yaw = pose
        vp = self.p.vehicle
        if contact is not None:
            cx, cy = contact
        else:
            direction = 1 if v_ref >= 0 else -1
            cx = (vp.wheelbase + vp.front_overhang + d + 0.4) if direction > 0 else -(vp.rear_overhang + d + 0.4)
            cy = 0.0
        bx = x + cx * math.cos(yaw) - cy * math.sin(yaw)
        by = y + cx * math.sin(yaw) + cy * math.cos(yaw)
        self.reflex.blocked_time = 0.0
        self.stats["replans"] += 1
        old_path, old_tracker = self.path, (self.tracker.path, self.tracker.seg_idx, self.tracker.wp)
        self.temp_boxes = [OBB(bx, by, yaw, 0.9, 0.9, "temp")]
        self._rebuild()
        self._plan(pose, budget=12000)             # a re-plan must be quick or give up
        self.temp_boxes = []
        if self.state != "DRIVE":                 # no way round: wait on the old path
            self.stats["plan_fail"] -= 1
            self.path = old_path
            self.tracker.set_path(old_tracker[0])
            self.tracker.seg_idx, self.tracker.wp = old_tracker[1], old_tracker[2]
            self.state = "DRIVE"
        self._rebuild_keep_path()

    def _rebuild_keep_path(self):
        path = self.path
        self._rebuild()
        self.path = path
