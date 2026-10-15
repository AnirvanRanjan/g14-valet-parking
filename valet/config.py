"""Shared parameters for the valet parking agent.

All units are SI (metres, radians, seconds) unless stated otherwise.
The same values are used by the Webots controller and by the headless
Python twin, so results from both are directly comparable.
"""
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class VehicleParams:
    """Ackermann car, dimensions close to the Webots BmwX5 PROTO.

    The reference point of the car is the centre of the rear axle (this is
    also the origin of Webots car PROTOs), so the kinematic bicycle model
    needs no offset.
    """
    wheelbase: float = 2.995          # L [m]
    width: float = 1.95               # body width [m]
    front_overhang: float = 0.95      # front axle -> front bumper [m]
    rear_overhang: float = 1.00       # rear axle  -> rear bumper  [m]
    max_steer: float = 0.55           # delta_max [rad]  (~31.5 deg)
    max_steer_rate: float = 1.2       # [rad/s] steering actuator limit
    max_speed: float = 3.0            # [m/s] forward speed limit in the lot (~11 km/h)
    max_reverse_speed: float = 1.2    # [m/s]
    max_accel: float = 1.5            # [m/s^2]
    speed_tau: float = 0.45           # first-order drivetrain time constant [s]

    @property
    def length(self) -> float:
        return self.wheelbase + self.front_overhang + self.rear_overhang

    @property
    def min_turn_radius(self) -> float:
        return self.wheelbase / math.tan(self.max_steer)

    @property
    def centre_offset(self) -> float:
        """Distance from the rear axle to the geometric centre of the body."""
        return (self.wheelbase + self.front_overhang - self.rear_overhang) / 2.0


@dataclass(frozen=True)
class LotParams:
    """Parking lot layout: 4 rows x 12 perpendicular slots, two aisles."""
    slot_width: float = 2.8
    slot_depth: float = 5.5
    slots_per_row: int = 12
    entry_lane_width: float = 6.0     # vertical lane on the left (x in [0, 6])
    aisle_width: float = 7.0

    @property
    def width(self) -> float:
        return self.entry_lane_width + self.slots_per_row * self.slot_width

    @property
    def height(self) -> float:
        return 4 * self.slot_depth + 2 * self.aisle_width


def _sensor_layout():
    """26 range sensors (Webots DistanceSensor), mounted 0.5 m high.

    * 7 per bumper: centre, two straight rays at y = +-0.5 m, and two corner
      sensors looking 30 deg and 60 deg outwards (so a 0.7 m wide pedestrian
      cannot hide between rays in front of / behind the car);
    * 6 per side, every 0.7 m from x = -0.4 to 3.1 m (rear-axle frame), so
      the side of the car is covered while it swings during a manoeuvre.
    Each entry: (name, x, y, mounting yaw) in the rear-axle frame.
    """
    xf, xr, hw = 3.94, -1.00, 0.97
    m = [("front_center", xf, 0.0, 0.0),
         ("front_left_in", xf - 0.02, 0.5, 0.0), ("front_right_in", xf - 0.02, -0.5, 0.0),
         ("front_left", 3.80, 0.85, math.radians(30)), ("front_right", 3.80, -0.85, math.radians(-30)),
         ("front_left_wide", 3.70, 0.92, math.radians(60)),
         ("front_right_wide", 3.70, -0.92, math.radians(-60)),
         ("rear_center", xr, 0.0, math.pi),
         ("rear_left_in", xr + 0.02, 0.5, math.pi), ("rear_right_in", xr + 0.02, -0.5, math.pi),
         ("rear_left", -0.90, 0.85, math.radians(150)), ("rear_right", -0.90, -0.85, math.radians(-150)),
         ("rear_left_wide", -0.80, 0.92, math.radians(120)),
         ("rear_right_wide", -0.80, -0.92, math.radians(-120))]
    for k, x in enumerate((-0.4, 0.3, 1.0, 1.7, 2.4, 3.1)):
        m.append((f"side_left_{k}", x, hw, math.radians(90)))
        m.append((f"side_right_{k}", x, -hw, math.radians(-90)))
    return tuple(m)


@dataclass(frozen=True)
class SensorParams:
    max_range: float = 8.0
    noise_std: float = 0.02           # [m]
    gps_noise_std: float = 0.05       # [m]
    heading_noise_std: float = math.radians(0.5)
    mounts: tuple = _sensor_layout()


@dataclass(frozen=True)
class PlannerParams:
    xy_resolution: float = 0.5        # Hybrid A* cell size [m]
    yaw_bins: int = 72                # 5 deg heading bins
    step: float = 1.0                 # motion primitive arc length [m]
    n_steer: int = 5                  # steering samples in [-max, max]
    steer_fraction: float = 0.88      # plan with 88 % of the steering limit -> margin for feedback
    reverse_cost: float = 1.6         # multiplier on reversing distance
    gear_switch_cost: float = 6.0     # penalty for each forward/reverse change
    steer_cost: float = 0.4           # penalty on |delta|
    steer_change_cost: float = 0.8    # penalty on |delta - delta_prev|
    safety_margin: float = 0.20       # extra clearance added to the footprint [m]
    max_expansions: int = 40000
    funnel_lateral_tol: float = 0.20  # goal-funnel acceptance (see hybrid_astar.py)
    funnel_heading_tol: float = math.radians(6.0)
    funnel_max_dist: float = 7.0
    funnel_min_dist: float = 2.5      # the final straight reverse is at least this long


@dataclass(frozen=True)
class ControlParams:
    dt: float = 0.05                  # control period [s] (Webots: 3 x 16 ms basic step ~ 48 ms)
    lookahead_min: float = 1.6        # pure pursuit L_d = max(min, k * v)
    lookahead_gain: float = 0.8
    stanley_k: float = 1.2
    stanley_soft: float = 0.5
    speed_kp: float = 1.8
    speed_ki: float = 0.6
    speed_kd: float = 0.05
    cruise_speed: float = 2.2         # [m/s] in the aisle (~8 km/h, typical AVP limit)
    manoeuvre_speed: float = 0.9      # [m/s] near cusps and inside the slot
    goal_pos_tol: float = 0.30        # parked if within this lateral/longitudinal error
    goal_yaw_tol: float = math.radians(5.0)


@dataclass(frozen=True)
class ReflexParams:
    stop_distance: float = 0.45       # R1 emergency stop [m] (distance along the swept path)
    swept_margin: float = 0.20        # body inflation for the swept-footprint check [m]
    ttc_safe: float = 2.5             # R2 time-to-collision horizon [s]
    fixed_slow_distance: float = 2.5  # baseline: fixed-distance stop
    dock_rear_gap: float = 0.25       # R3 hard stop when rear sensor reads this [m]
    centring_gain: float = 0.12       # R3 lateral centring gain [rad/m]
    blocked_replan_time: float = 4.0  # R4 [s]


@dataclass(frozen=True)
class Params:
    vehicle: VehicleParams = field(default_factory=VehicleParams)
    lot: LotParams = field(default_factory=LotParams)
    sensors: SensorParams = field(default_factory=SensorParams)
    planner: PlannerParams = field(default_factory=PlannerParams)
    control: ControlParams = field(default_factory=ControlParams)
    reflex: ReflexParams = field(default_factory=ReflexParams)


DEFAULT = Params()
