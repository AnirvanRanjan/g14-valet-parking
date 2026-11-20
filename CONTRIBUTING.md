# Contributing

Thanks for contributing to the Autonomous Valet Parking Agent.

This repository contains planning, control, localisation, safety, simulation, experiment, and Webots-integration components. Changes should preserve reproducibility and avoid unintentionally altering behaviour outside their intended scope.

## Development Setup

Clone the repository and install the dependencies:

```bash
git clone https://github.com/AnirvanRanjan/g14-valet-parking.git
cd g14-valet-parking
pip install -r requirements.txt
```

## Development Workflow

1. Create a branch from the latest `main`.
2. Make focused changes related to one feature, fix, or documentation improvement.
3. Run the test suite locally.
4. Commit the changes with a clear commit message.
5. Open a pull request describing what changed and how it was validated.

Example branch names:

```text
feature/planner-improvement
fix/webots-controller
docs/readme-update
test/reflex-layer
```

## Testing

Before opening a pull request, run:

```bash
python -m unittest discover -s tests -v
```

The test suite covers components including:

- motion planning
- path tracking and control
- localisation
- cost-map and geometry utilities
- reflexive safety behaviour
- mission-level integration
- Webots controller behaviour using the fake Driver

All existing tests should pass before changes are merged.

## Simulation Validation

For changes affecting agent behaviour, run a headless mission when appropriate:

```bash
python sim/run_demo.py --seed 3 --pedestrians 2
```

For experiment-related changes, use the reproducible experiment pipeline:

```bash
python experiments/run_experiments.py --n 30
python experiments/make_figures.py
```

## Pull Requests

Keep pull requests focused and include:

- a concise summary of the change
- the reason for the change
- affected components
- validation or test results
- screenshots, figures, or logs when relevant

Avoid mixing unrelated refactors, documentation changes, and behavioural changes in the same pull request.

## Code and Project Conventions

- Preserve deterministic seeds where reproducibility matters.
- Keep shared autonomous-agent logic inside `valet/` rather than duplicating it between Webots and the headless simulator.
- Update tests when behaviour changes.
- Update documentation when commands, configuration, or project behaviour changes.
- Do not commit generated temporary files, local environments, or credentials.

## Reporting Problems

Use GitHub Issues for reproducible bugs, unexpected simulation behaviour, documentation problems, or proposed improvements.

When reporting simulation problems, include the relevant seed, configuration, controller/planner selection, and error output whenever possible.
