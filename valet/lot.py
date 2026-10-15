"""Parking-lot model and random scenario generator.

Layout (x to the right, y up, origin at the lower-left corner):

    y=36  +-----------------------------------------------+
          |      |  D0  D1  ...                 D11 (open -y)
          |      |-----------------------------------------
          | entry|            aisle 2 (7 m)
          | lane |-----------------------------------------
          |      |  C0  C1  ...                 C11 (open +y)
          | 6 m  |  B0  B1  ...                 B11 (open -y)
          |      |-----------------------------------------
          |      |            aisle 1 (7 m)
          |      |-----------------------------------------
          |  ^   |  A0  A1  ...                 A11 (open +y)
    y=0   +--|--------------------------------------------+
           entrance                                   x=39.6

Each slot stores the target parking pose of the rear-axle centre for a
reverse-in (nose-out) manoeuvre, which is the valet policy used here.
"""
from dataclasses import dataclass, field
import math
import random
import numpy as np

from .config import DEFAULT, Params
from .geometry import OBB


@dataclass
class Slot:
    sid: str
    row: str
    index: int
    cx: float               # slot rectangle centre
    cy: float
    opening: int            # +1 if the slot opens towards +y, -1 towards -y
    occupied: bool = False

    def park_pose(self, p: Params = DEFAULT):
        """Rear-axle pose for a nose-out (reverse-in) park."""
        vp, lp = p.vehicle, p.lot
        yaw = math.pi / 2 if self.opening > 0 else -math.pi / 2
        back_wall_y = self.cy - self.opening * lp.slot_depth / 2
        y = back_wall_y + self.opening * (vp.rear_overhang + 0.40)
        return (self.cx, y, yaw)

    def entry_pose(self, p: Params = DEFAULT, extra: float = 0.0):
        """Pose on the slot axis with the rear axle at the slot mouth + extra."""
        x, y, yaw = self.park_pose(p)
        mouth_y = self.cy + self.opening * p.lot.slot_depth / 2
        y_entry = mouth_y + self.opening * (extra + 0.5)
        return (self.cx, y_entry, yaw)

    def rect(self, p: Params = DEFAULT) -> OBB:
        return OBB(self.cx, self.cy, math.pi / 2, p.lot.slot_depth, p.lot.slot_width, "slot")


@dataclass
class Pedestrian:
    """Walks back and forth between A and B. A simple courtesy rule makes the
    pedestrian wait instead of stepping into a car that is directly in front
    of them (people do not walk into the side of a car), so every collision
    that remains is one the car could have avoided."""
    ax: float
    ay: float
    bx: float
    by: float
    speed: float = 1.2
    start_delay: float = 0.0
    radius: float = 0.35
    s: float = 0.0                  # progress along the 2*|AB| loop [m]
    wait: float = 0.0               # how long the pedestrian has been waiting [s]
    gone: bool = False              # stepped out of the aisle (between parked cars)

    def _at(self, s):
        d = math.hypot(self.bx - self.ax, self.by - self.ay)
        s = s % (2 * d)
        f = s / d if s <= d else 2 - s / d
        return (self.ax + f * (self.bx - self.ax), self.ay + f * (self.by - self.ay))

    def position(self, t=None):
        return (-100.0, -100.0) if self.gone else self._at(self.s)

    def step(self, t, dt, blocked_fn=None):
        if t < self.start_delay or self.gone:
            return
        nxt = self._at(self.s + self.speed * dt)
        if blocked_fn is not None and blocked_fn(nxt, self.position()):
            self.wait += dt
            if self.wait > 1.5:     # give way: turn round and walk back ...
                d = math.hypot(self.bx - self.ax, self.by - self.ay)
                at_end = min(self.s % d, d - self.s % d) < 0.3
                if at_end:          # ... or, at the edge of the aisle, step between the cars
                    self.gone = True
                self.s = (2 * d - self.s) % (2 * d)
                self.wait = 0.0
            return
        self.wait = 0.0
        self.s += self.speed * dt


@dataclass
class Scenario:
    slots: list
    parked: list                     # OBBs of parked cars
    walls: list                      # OBBs of walls / kerbs
    start: tuple                     # ego start pose (x, y, yaw)
    exit_point: tuple                # where the valet hands the car back
    pedestrians: list = field(default_factory=list)
    seed: int = 0
    steal_event: tuple = None        # (time, slot id) another car takes a slot

    @property
    def obstacles(self):
        return self.walls + self.parked

    def free_slots(self):
        return [s for s in self.slots if not s.occupied]

    def slot(self, sid):
        return next(s for s in self.slots if s.sid == sid)


def build_slots(p: Params = DEFAULT):
    lp = p.lot
    rows = [  # (name, centre y, opening direction)
        ("A", lp.slot_depth / 2, +1),
        ("B", lp.slot_depth + lp.aisle_width + lp.slot_depth / 2, -1),
        ("C", 2 * lp.slot_depth + lp.aisle_width + lp.slot_depth / 2, +1),
        ("D", 3 * lp.slot_depth + 2 * lp.aisle_width + lp.slot_depth / 2, -1),
    ]
    slots = []
    for name, cy, opening in rows:
        for i in range(lp.slots_per_row):
            cx = lp.entry_lane_width + (i + 0.5) * lp.slot_width
            slots.append(Slot(f"{name}{i}", name, i, cx, cy, opening))
    return slots


def build_walls(p: Params = DEFAULT):
    lp = p.lot
    W, H, t = lp.width, lp.height, 0.3
    walls = [
        OBB(W / 2, -t / 2, 0.0, W + 2 * t, t, "wall"),            # bottom
        OBB(W / 2, H + t / 2, 0.0, W + 2 * t, t, "wall"),         # top
        OBB(-t / 2, H / 2, 0.0, t, H, "wall"),                    # left
        OBB(W + t / 2, H / 2, 0.0, t, H, "wall"),                 # right
    ]
    # Kerb between the back-to-back rows B and C (low island, 0.3 m wide).
    y_mid = 2 * lp.slot_depth + lp.aisle_width
    walls.append(OBB(lp.entry_lane_width + lp.slots_per_row * lp.slot_width / 2, y_mid, 0.0,
                     lp.slots_per_row * lp.slot_width, 0.3, "wall"))
    return walls


def parked_car_box(slot: Slot, rng: random.Random, p: Params = DEFAULT) -> OBB:
    """A parked car placed in a slot with a small random offset and angle."""
    lp = p.lot
    length, width = rng.uniform(4.3, 4.9), rng.uniform(1.75, 1.95)
    dx = rng.uniform(-0.2, 0.2)
    back = slot.cy - slot.opening * lp.slot_depth / 2
    cy = back + slot.opening * (0.3 + length / 2 + rng.uniform(0.0, 0.3))
    yaw = math.pi / 2 + rng.uniform(-0.05, 0.05)
    return OBB(slot.cx + dx, cy, yaw, length, width, "car")


def make_scenario(seed: int = 0, occupancy: float = 0.7, n_pedestrians: int = 0,
                  steal: bool = False, p: Params = DEFAULT) -> Scenario:
    """Random lot occupancy (fraction of slots taken) and optional pedestrians."""
    rng = random.Random(seed)
    slots = build_slots(p)
    n_occ = int(round(occupancy * len(slots)))
    for s in rng.sample(slots, n_occ):
        s.occupied = True
    parked = [parked_car_box(s, rng, p) for s in slots if s.occupied]
    lp = p.lot
    start = (lp.entry_lane_width / 2, 3.0, math.pi / 2)
    # Drop-off at the entrance (bottom-left), pick-up zone at the top-left:
    # the valet must later drive the car from its slot to the pick-up zone.
    exit_point = (lp.entry_lane_width / 2, lp.height - 3.0)
    peds = []
    aisles_y = [lp.slot_depth + lp.aisle_width / 2,
                3 * lp.slot_depth + 1.5 * lp.aisle_width]
    for k in range(n_pedestrians):
        # Pedestrians cross an aisle from one row of cars to the other.
        ay = aisles_y[k % 2]
        x = rng.uniform(lp.entry_lane_width + 4, lp.width - 4)
        peds.append(Pedestrian(x, ay - lp.aisle_width / 2 + 0.4, x + rng.uniform(-1, 1),
                               ay + lp.aisle_width / 2 - 0.4,
                               speed=rng.uniform(0.9, 1.4), start_delay=rng.uniform(0, 6)))
    sc = Scenario(slots, parked, build_walls(p), start, exit_point, peds, seed)
    if steal:
        sc.steal_event = (rng.uniform(4.0, 8.0), None)   # slot chosen at run time
    return sc
