"""Runs the real Webots controller file (controllers/valet_parking) against a
fake `vehicle.Driver` that is backed by the Python twin. This checks the
controller glue code (devices, argument parsing, unit conversions, steering
sign, logging) without a Webots installation.
"""
import importlib.util
import math
import os
import sys
import tempfile
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from valet.config import DEFAULT                     # noqa: E402
from valet.lot import make_scenario                  # noqa: E402
from sim.simulator import Simulator                  # noqa: E402


class _Dev:
    def __init__(self, fn):
        self.fn = fn

    def enable(self, _):
        pass

    def getValues(self):
        return self.fn()

    def getRollPitchYaw(self):
        return self.fn()

    def getValue(self):
        return self.fn()


class FakeDriver:
    """Minimal subset of the Webots automobile Driver API."""
    instance = None

    def __init__(self):
        FakeDriver.instance = self
        self.sim = Simulator(make_scenario(seed=4, occupancy=0.7), seed=4)
        self.t, self.steps, self.cruise, self.steer = 0.0, 0, 0.0, 0.0
        self._ranges = self.sim.ranges()

    def getBasicTimeStep(self):
        return 16.0

    def getTime(self):
        return self.t

    def getDevice(self, name):
        car = self.sim.car
        if name == "gps":
            return _Dev(lambda: [car.x, car.y, 0.4])
        if name == "imu":
            return _Dev(lambda: [0.0, 0.0, car.yaw])
        return _Dev(lambda n=name: self._ranges[n])

    def setSteeringAngle(self, a):
        self.steer = -a                    # Webots: positive = right

    def setCruisingSpeed(self, kmh):
        self.cruise = kmh / 3.6

    def setBrakeIntensity(self, b):
        pass

    def step(self):
        if self.steps > 6000 or getattr(self, "finished", False):
            return -1
        dt = 0.016
        car = self.sim.car
        car.step(self.steer, (self.cruise - car.v) / 0.5, dt)    # crude cruise control
        self.t += dt
        self.steps += 1
        self._ranges = self.sim.ranges()
        return 0


class TestWebotsController(unittest.TestCase):
    def test_controller_parks_car(self):
        vehicle = types.ModuleType("vehicle")
        vehicle.Driver = FakeDriver
        sys.modules["vehicle"] = vehicle
        # the controller reads worlds/lot_config.json -> write one for seed 4
        sc = FakeDriver().sim.sc
        cfg_path = os.path.join(ROOT, "worlds", "lot_config.json")
        backup = None
        if os.path.exists(cfg_path):
            with open(cfg_path) as f:
                backup = f.read()
        import json
        with open(cfg_path, "w") as f:
            json.dump({"seed": 4, "occupancy": 0.7, "start": sc.start, "exit_point": sc.exit_point,
                       "occupied": [s.sid for s in sc.slots if s.occupied],
                       "parked": [[b.cx, b.cy, b.yaw, b.length, b.width] for b in sc.parked],
                       "walls": [[w.cx, w.cy, w.yaw, w.length, w.width] for w in sc.walls]}, f)
        try:
            spec = importlib.util.spec_from_file_location(
                "valet_parking", os.path.join(ROOT, "controllers", "valet_parking", "valet_parking.py"))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            sys.argv = ["valet_parking"]
            drv, agent = mod.main()
            car = drv.sim.car
            self.assertLess(drv.steps, 6000, "controller did not finish in time")
            self.assertEqual(agent.state, "PARKED")
            self.assertIsNone(drv.sim.check_collision())
            gx, gy, gyaw = agent.target.park_pose()
            self.assertLess(math.hypot(car.x - gx, car.y - gy), 0.4)
            self.assertLess(abs(math.degrees(car.yaw - gyaw)), 6.0)
        finally:
            if backup is not None:
                with open(cfg_path, "w") as f:
                    f.write(backup)
            sys.modules.pop("vehicle", None)


if __name__ == "__main__":
    unittest.main()
