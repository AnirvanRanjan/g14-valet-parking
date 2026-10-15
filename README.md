# Autonomous Valet Parking Agent (P13, Group G14)

CSAI4014 Algorithms for Intelligent Systems and Robotics, B.Tech CSE (AI & ML), Semester VII, Batch B2, UPES.

A car-like (Ackermann) vehicle in **Webots** picks a free slot in a 48-slot parking lot, plans a
drivable route with **Hybrid A\***, drives it with **Pure Pursuit + PID**, reverses into the slot
nose-out, and stays safe around pedestrians using a **reflexive swept-footprint time-to-collision layer**.
The three layers follow the *declarative - procedural - reflexive* hierarchy from the AISR syllabus.

| Member | SAP ID | Module (main files) |
|---|---|---|
| Saurabh Kumar | 500124288 | Lot model, Webots world and controller: `valet/lot.py`, `valet/costmap.py`, `tools/make_world.py`, `controllers/valet_parking/` |
| Naman Chanana | 500119485 | Deliberative planners: `valet/planning/hybrid_astar.py`, `valet/planning/grid_astar.py`, `valet/planning/path.py` |
| Anirvan Ranjan | 500119330 | Agent architecture, slot-selection utility, mission FSM, localisation: `valet/agent.py`, `valet/slot_selection.py`, `valet/localisation.py` |
| Aditya Pratap Singh Sisodia | 500121985 | Procedural control: `valet/control/` (Pure Pursuit, Stanley, PID, speed profile, tracker) |
| Priyal Sharma | 500122041 | Reflexive layer, simulator twin, experiments: `valet/reflex.py`, `sim/`, `experiments/` |

## Architecture

```
            +----------------------- ValetAgent (valet/agent.py) -----------------------+
 GPS, IMU,  | DECLARATIVE  slot selection (utility)  ->  Hybrid A* + goal funnel        |
 wheel spd  |              (once per mission, or on events: slot taken / blocked)        |
 -> Kalman  |------------------------------------------------------------------------------|
   filter   | PROCEDURAL   Pure Pursuit / Stanley steering, speed profile + PID, cusps    |  -> steering
            |------------------------------------------------------------------------------|  -> speed
 26 range   | REFLEXIVE    R1 e-stop, R2 swept-footprint TTC, R3 docking, R4 blocked      |
 sensors -> |              (every 48 ms, overrides the layers above)                      |
            +-------------------------------------------------------------------------------+
 Mission FSM: SELECT -> PLAN -> DRIVE -> DOCK -> PARKED   (re-select / re-plan on events)
```

## Quick start

```bash
pip install -r requirements.txt

# 1. headless twin: one mission, animated (writes results/demo.gif)
python sim/run_demo.py --seed 3 --pedestrians 2

# 2. unit + integration tests (includes the Webots controller against a fake Driver)
python -m unittest discover -s tests -v

# 3. all experiments of the report (~15-25 min on 8 cores) and all figures/tables
python experiments/run_experiments.py --n 30
python experiments/make_figures.py
```

### Running in Webots (R2025a or newer)

```bash
python tools/make_world.py --seed 3 --occupancy 0.7 --pedestrians 1
# optional: --planner grid  --law stanley  --reflex fixed  --selector nearest
```
Then open `worlds/valet_parking.wbt` in Webots and press Run. The controller
`controllers/valet_parking/valet_parking.py` prints the mission states in the Webots console
and writes a CSV log to `results/webots_runs/`. Webots must use a Python 3.9+ interpreter
that has NumPy and SciPy installed (Webots > Preferences > Python command).

Troubleshooting:
* **Car turns the wrong way at the first bend.** Add `"--steer-sign=1"` to `controllerArgs`
  in the world file. Webots uses positive = right; the agent uses positive = left.
* **"sensors reading ~0 m" warning.** That sensor sees the car's own body. Move it 3-5 cm
  outwards in `valet/config.py` (`_sensor_layout`) and run `make_world.py` again.
* **Car does not reverse in cruise mode.** Run with `"--speed-mode=throttle"`, which uses gears
  and our PID for the throttle and brake.

## Repository layout

```
valet/                 shared agent code (used by Webots AND the twin)
  config.py            all parameters (vehicle, lot, sensors, planner, control, reflex)
  geometry.py          oriented boxes, ray casting, SAT collision tests
  lot.py               parking-lot model, slots, pedestrians, random scenarios
  costmap.py           occupancy grid + distance transform, footprint check
  planning/            grid A* (baseline), Hybrid A* + goal funnel, Path
  control/             PID, Pure Pursuit, Stanley, PathTracker (procedural layer)
  reflex.py            reflexive layer (R1-R4)
  slot_selection.py    nearest-slot vs utility-based slot choice
  localisation.py      GPS + odometry Kalman filter
  agent.py             three-layer agent + mission FSM
sim/                   headless twin (bicycle model, sensors, pedestrians), demo animation
controllers/valet_parking/   Webots controller
tools/make_world.py    generates worlds/valet_parking.wbt from the same lot model
experiments/           Monte Carlo experiments + figure/table generation
results/               experiments.csv, figures/, tables/, summary.md
tests/                 unittest suite
docs/                  meeting log, commit plan, video script, viva preparation
```

## Main results (headless twin, 30 random lots per setting)

See `results/summary.md` and Section 9 of the report. All numbers are reproducible with the
commands above: every mission uses a fixed seed.

## References

Dolgov et al. (2010) Hybrid A\*; Coulter (1992) Pure Pursuit; Thrun et al. (2006) Stanley;
Brooks (1986) subsumption; Gat (1998) three-layer architectures; ISO 23374-1:2023 AVP systems.
The full IEEE list is in the report.
