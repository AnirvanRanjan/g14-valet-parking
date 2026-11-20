# Autonomous Valet Parking Agent (P13, Group G14)

![Python](https://img.shields.io/badge/Python-3.9%2B-blue?logo=python)
![Webots](https://img.shields.io/badge/Webots-R2025a%2B-orange)
![Planner](https://img.shields.io/badge/Planning-Hybrid%20A*-purple)
![Tests](https://img.shields.io/badge/Tests-unittest-green)

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

## Key Features

- **Hybrid A\*** motion planning for car-like vehicle constraints
- **Pure Pursuit / Stanley** path tracking with PID speed control
- Utility-based parking-slot selection
- Three-layer **declarative–procedural–reflexive** agent architecture
- Swept-footprint time-to-collision pedestrian safety
- GPS + odometry Kalman-filter localisation
- Mission FSM with re-selection and re-planning
- Headless simulation twin for reproducible experiments
- Webots integration with generated parking scenarios
- Automated unit and integration test suite

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

## Quick Start

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Run the headless simulator

Run one animated parking mission:

```bash
python sim/run_demo.py --seed 3 --pedestrians 2
```

The generated animation is saved to:

```text
results/demo.gif
```

### 3. Run the test suite

Run the unit and integration tests:

```bash
python -m unittest discover -s tests -v
```

The test suite also includes the Webots controller running against a fake Driver.

### 4. Reproduce the experiments

Run the experiments used for the project report:

```bash
python experiments/run_experiments.py --n 30
python experiments/make_figures.py
```

The complete experiment suite takes approximately **15–25 minutes on an 8-core machine**.

### Running in Webots

Requires **Webots R2025a or newer**.

Generate the parking world:

```bash
python tools/make_world.py --seed 3 --occupancy 0.7 --pedestrians 1
```

Optional alternatives include:

```text
--planner grid
--law stanley
--reflex fixed
--selector nearest
```

Then open:

```text
worlds/valet_parking.wbt
```

in Webots and press **Run**.

The controller `controllers/valet_parking/valet_parking.py` prints mission states in the Webots console and writes CSV logs to:

```text
results/webots_runs/
```

Webots must use a Python 3.9+ interpreter with NumPy and SciPy installed:

**Webots → Preferences → Python command**

### Troubleshooting

- **Car turns the wrong way at the first bend:** add `"--steer-sign=1"` to `controllerArgs` in the world file. Webots uses positive = right; the agent uses positive = left.
- **`sensors reading ~0 m` warning:** the sensor may see the car's own body. Move it 3–5 cm outward in `valet/config.py` (`_sensor_layout`) and regenerate the world with `make_world.py`.
- **Car does not reverse in cruise mode:** run with `"--speed-mode=throttle"`, which uses gears and the project's PID controller for throttle and braking.


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

## Results

The system was evaluated in the headless simulation twin across **30 random parking-lot scenarios per setting**.

The experiments evaluate the main components of the autonomous valet pipeline, including planning, control, slot selection, and reflexive safety behavior.

All experiments are reproducible using fixed random seeds.

Detailed results are available in:

- [`results/summary.md`](results/summary.md) — experiment summary
- `results/figures/` — generated plots and visualisations
- `results/tables/` — generated result tables
- `results/experiments.csv` — raw experiment data
- `report/` — complete project report and analysis

To reproduce the experiments:

```bash
python experiments/run_experiments.py --n 30
python experiments/make_figures.py

## References

Dolgov et al. (2010) Hybrid A\*; Coulter (1992) Pure Pursuit; Thrun et al. (2006) Stanley;
Brooks (1986) subsumption; Gat (1998) three-layer architectures; ISO 23374-1:2023 AVP systems.
The full IEEE list is in the report.
