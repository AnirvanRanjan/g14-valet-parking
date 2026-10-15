# GitHub plan: who commits what (B1 traceability)

The rubric (B1, 15 marks) needs every member's contribution to be **traceable to commits from
their own GitHub account**. Do not let one person push everything.

## 1. Set up the repository (team lead, 10 minutes)

```bash
# team lead (Saurabh) creates an EMPTY repo on GitHub: g14-valet-parking (public or shared with faculty)
git clone https://github.com/<lead>/g14-valet-parking.git
cd g14-valet-parking
# add the team-mates: Settings > Collaborators > add all four
```

Each member then sets their identity once, on their own laptop:
```bash
git config --global user.name  "Full Name"
git config --global user.email "the-email-of-your-github-account"
```

## 2. Commit order (each member commits from their own machine and account)

Do the steps in this order, because later modules import earlier ones. Each member copies
their files from this folder into their clone, then commits and pushes. Pull before you start.

| Step | Member | Files to add | Commit message |
|---|---|---|---|
| 1 | Saurabh | `README.md`, `requirements.txt`, `.gitignore`, `valet/__init__.py`, `valet/config.py`, `valet/geometry.py` | `Project skeleton, parameters and geometry toolkit` |
| 2 | Saurabh | `valet/lot.py`, `valet/costmap.py` | `Parking-lot model, random scenarios and costmap` |
| 3 | Naman | `valet/planning/__init__.py`, `valet/planning/path.py`, `valet/planning/grid_astar.py` | `Grid A* baseline planner and Path class` |
| 4 | Naman | `valet/planning/hybrid_astar.py` | `Hybrid A* with Ackermann primitives and goal funnel` |
| 5 | Aditya | `valet/control/__init__.py`, `valet/control/pid.py`, `valet/control/steering.py` | `PID, Pure Pursuit and Stanley (forward and reverse)` |
| 6 | Aditya | `valet/control/tracker.py` | `Path tracker: segments, cusps, speed profile` |
| 7 | Priyal | `valet/reflex.py` | `Reflexive layer: e-stop, swept-footprint TTC, docking` |
| 8 | Anirvan | `valet/slot_selection.py`, `valet/localisation.py` | `Utility-based slot selection and GPS/odometry Kalman filter` |
| 9 | Anirvan | `valet/agent.py` | `Three-layer valet agent and mission state machine` |
| 10 | Priyal | `sim/__init__.py`, `sim/simulator.py`, `sim/run_demo.py` | `Headless simulator twin and demo animation` |
| 11 | Saurabh | `tools/make_world.py`, `worlds/`, `controllers/valet_parking/valet_parking.py` | `Webots world generator and Webots controller` |
| 12 | Priyal | `experiments/run_experiments.py` | `Monte Carlo experiment runner (E1-E7)` |
| 13 | Naman | `tests/__init__.py`, `tests/test_valet.py` | `Unit and integration tests` |
| 14 | Aditya | `tests/test_webots_controller.py` | `Webots controller test with a fake Driver` |
| 15 | Priyal | `experiments/make_figures.py`, `results/` | `Figures, tables and experiment results` |
| 16 | Anirvan | `docs/` | `Meeting log, video script and viva notes` |
| 17 | everyone | the report `.tex` / PDF and slides, in `report/` | `Report section X` (each member commits their own sections) |

Rules that protect your marks:
* Every member makes **at least 3 commits** (the table gives each member 3-4).
* Use clear commit messages, like the ones above. Never "update" or "final".
* After step 11, everyone runs `python -m unittest discover -s tests` and fixes their own module if
  a test fails.
* Do not rewrite history (`git push --force`). The evaluator reads the commit log.
* While you build, change and improve your own module in small commits. More real commits
  over several days look better than one big dump.
