"""Unit and integration tests.  Run:  python -m unittest discover -s tests -v
(or `pytest tests/` if pytest is installed)."""
import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from valet.config import DEFAULT                                   # noqa: E402
from valet.geometry import OBB, obb_overlap, ray_cast_boxes        # noqa: E402
from valet.costmap import CostMap                                  # noqa: E402
from valet.lot import make_scenario                                # noqa: E402
from valet.planning import HybridAStarPlanner, GridAStarPlanner, Path  # noqa: E402
from valet.control import PID, PurePursuit, Stanley                # noqa: E402
from valet.control.steering import nearest_index, cross_track      # noqa: E402
from valet.reflex import ReflexLayer                               # noqa: E402
from valet.localisation import PoseFilter                          # noqa: E402
from sim.simulator import Simulator, CarModel                      # noqa: E402

P = DEFAULT


class TestGeometry(unittest.TestCase):
    def test_overlap(self):
        a = OBB(0, 0, 0, 4, 2)
        self.assertTrue(obb_overlap(a, OBB(2.8, 0, 0.3, 2, 2)))
        self.assertFalse(obb_overlap(a, OBB(5.1, 0, 0, 2, 2)))
        self.assertFalse(obb_overlap(a, OBB(0, 2.5, math.pi / 4, 1, 1)))

    def test_ray_cast(self):
        box = OBB(5, 0, 0, 2, 2)
        d = ray_cast_boxes([0.0], [0.0], np.array([[1.0, 0.0]]), [box], 8.0)
        self.assertAlmostEqual(d[0], 4.0, places=6)
        d = ray_cast_boxes([0.0], [0.0], np.array([[0.0, 1.0]]), [box], 8.0)
        self.assertEqual(d[0], 8.0)


class TestCostMap(unittest.TestCase):
    def setUp(self):
        self.sc = make_scenario(seed=1, occupancy=0.7)
        self.cm = CostMap(self.sc.obstacles)

    def test_park_pose_free(self):
        for s in self.sc.free_slots():
            self.assertTrue(self.cm.poses_free(*s.park_pose(), margin=0.05), s.sid)

    def test_pose_on_parked_car_blocked(self):
        car = self.sc.parked[0]
        self.assertFalse(self.cm.poses_free(car.cx - 1.5, car.cy, car.yaw))


class TestPlanners(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sc = make_scenario(seed=3, occupancy=0.7)
        cls.cm = CostMap(cls.sc.obstacles)
        cls.slot = cls.sc.free_slots()[0]

    def test_hybrid_path_is_drivable_and_free(self):
        path, info = HybridAStarPlanner(self.cm).plan(self.sc.start, self.slot.park_pose())
        self.assertTrue(info["ok"])
        kmax = math.tan(P.vehicle.max_steer) / P.vehicle.wheelbase
        # primitives are exact arcs; allow the short funnel tail some slack
        self.assertLessEqual(np.percentile(path.curvature(), 95), kmax * 1.05)
        self.assertTrue(self.cm.poses_free(path.x, path.y, path.yaw, margin=0.0))
        gx, gy, gyaw = self.slot.park_pose()
        self.assertLess(math.hypot(path.x[-1] - gx, path.y[-1] - gy), 0.05)
        self.assertEqual(path.direction[-1], -1)          # reverse-in (nose-out) policy

    def test_funnel_reduces_expansions(self):
        _, a = HybridAStarPlanner(self.cm, use_funnel=True).plan(self.sc.start, self.slot.park_pose())
        _, b = HybridAStarPlanner(self.cm, use_funnel=False).plan(self.sc.start, self.slot.park_pose())
        self.assertLess(a["expansions"], b["expansions"])

    def test_grid_astar_finds_route(self):
        path, info = GridAStarPlanner(self.cm).plan(self.sc.start, self.slot)
        self.assertTrue(info["ok"])
        self.assertGreater(path.length, 5.0)


class TestControl(unittest.TestCase):
    def _track(self, law_cls, direction):
        xs = np.arange(0, 20, 0.25) * direction
        path = Path(xs, np.zeros_like(xs), np.zeros_like(xs), np.full(len(xs), direction))
        car, law, i = CarModel((0, 0.5, 0.1)), law_cls(P.vehicle, P.control), 0
        for _ in range(250):
            i = nearest_index(path, car.x, car.y, i)
            car.step(law(path, i, car.x, car.y, car.yaw, car.v), (0.8 * direction - car.v) * 2, 0.05)
        return abs(cross_track(path, i, car.x, car.y))

    def test_pure_pursuit_converges_both_directions(self):
        self.assertLess(self._track(PurePursuit, 1), 0.05)
        self.assertLess(self._track(PurePursuit, -1), 0.05)

    def test_stanley_converges_both_directions(self):
        self.assertLess(self._track(Stanley, 1), 0.15)
        self.assertLess(self._track(Stanley, -1), 0.15)

    def test_pid_speed_step(self):
        pid, v, a, dt = PID(1.8, 0.6, 0.05, 0.05, -3, 1.5), 0.0, 0.0, 0.05
        for _ in range(200):
            u = pid(2.0, v)
            a += (u - a) * dt / P.vehicle.speed_tau
            v += a * dt
        self.assertAlmostEqual(v, 2.0, delta=0.05)


class TestReflex(unittest.TestCase):
    def test_stops_for_obstacle_ahead_not_beside(self):
        rl = ReflexLayer(P, "ttc")
        ranges = {m[0]: 8.0 for m in P.sensors.mounts}
        ranges["front_center"] = 0.3
        _, v, info = rl.apply(0.0, 2.0, 2.0, ranges, 0.05)
        self.assertEqual(v, 0.0)
        self.assertEqual(info["rule"], "R1-estop")
        rl = ReflexLayer(P, "ttc")
        ranges = {m[0]: 8.0 for m in P.sensors.mounts}
        ranges["side_left_2"] = 0.6            # parked car beside us, driving straight
        _, v, _ = rl.apply(0.0, 2.0, 2.0, ranges, 0.05)
        self.assertEqual(v, 2.0)

    def test_side_obstacle_matters_when_turning(self):
        rl = ReflexLayer(P, "ttc")
        ranges = {m[0]: 8.0 for m in P.sensors.mounts}
        ranges["side_left_5"] = 0.25           # near the front-left wheel
        _, v_straight, _ = rl.apply(0.0, 1.0, 1.0, ranges, 0.05)
        rl = ReflexLayer(P, "ttc")
        _, v_turn, _ = rl.apply(0.45, 1.0, 1.0, ranges, 0.05)
        self.assertGreater(v_straight, v_turn)


class TestLocalisation(unittest.TestCase):
    def test_filter_reduces_error(self):
        rng = np.random.default_rng(0)
        f, x, raw, filt = PoseFilter(), 0.0, [], []
        for _ in range(400):
            x += 1.0 * 0.05
            zx, zy = x + rng.normal(0, 0.2), rng.normal(0, 0.2)
            ex, ey = f.update(zx, zy, 0.0, 1.0 + rng.normal(0, 0.02), 0.05)
            raw.append(math.hypot(zx - x, zy)); filt.append(math.hypot(ex - x, ey))
        self.assertLess(np.sqrt(np.mean(np.square(filt[50:]))),
                        0.5 * np.sqrt(np.mean(np.square(raw[50:]))))


class TestMission(unittest.TestCase):
    def test_full_mission_parks(self):
        r = Simulator(make_scenario(seed=2, occupancy=0.7), seed=2).run()
        self.assertTrue(r["success"], r)
        self.assertFalse(r["collision"])

    def test_slot_taken_event_triggers_reselection(self):
        r = Simulator(make_scenario(seed=1, occupancy=0.7, steal=True), seed=1).run()
        self.assertEqual(r["reselections"], 1)
        self.assertTrue(r["success"], r)


if __name__ == "__main__":
    unittest.main()
