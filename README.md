# amr-stack

A self-contained, pip-installable Python package implementing a complete autonomous mobile robot
software stack in pure Python (no ROS, no scipy).

**Version:** 0.1.0  
**Python:** 3.8.10  **numpy:** 1.17.4  **GUI:** tkinter (stdlib)

---

## Capabilities

| Capability | Delivered by | Proven by |
|---|---|---|
| Mapping | `amr.mapping` — log-odds occupancy grid + ROS-format map I/O | `test_mapper.py`, `test_map_io.py` |
| Configuration via file | `amr.core.config` + `configs/*.yaml` + `--set` overrides | `test_config.py` |
| Logging | `amr.core.log` — console + rotating file + per-module levels + GUI handler | `test_log.py` |
| Planning | `amr.planning` — inflated costmap + A* global + DWA local planner | `test_costmap.py`, `test_astar.py`, `test_dwa.py` |
| Localization | `amr.localization` — MCL particle filter, likelihood-field sensor model | `test_mcl.py` |
| SLAM | `amr.slam` — correlative scan-matching SLAM + log-odds map | `test_slam.py` |
| Simulation | `amr.sim` — diff-drive kinematics, vectorized lidar, odom noise, collisions | `test_sim.py` |
| Deployment | `deploy/` — Dockerfile, docker-compose, systemd unit, install.sh | Task 4.2 static checks |
| Setup | `scripts/setup.sh`, `Makefile`, `pyproject.toml` | Gate A |
| GUI / HRI | `amr.gui` — tkinter console: live map, particles, path, click-to-goal, teleop, e-stop, log | `test_gui_smoke.py` |

---

## Quickstart

```bash
# 1. Create the venv and install the package
make setup

# 2. Run the full test suite (43 tests)
make test

# 3. Run the full autonomy demo:
#    SLAM a mission → save map → navigate two goals → print DEMO PASS
make demo

# 4. Launch the interactive HRI GUI (requires a display)
make gui
```

The setup script handles Ubuntu 20.04's broken `ensurepip` automatically via a `get-pip.py`
bootstrap. System numpy and PyYAML are reused via `--system-site-packages`; only pytest is
fetched from PyPI.

---

## CLI Reference

All commands share these global flags:

```
amr [--config PATH] [--set KEY=VAL ...] [--log-level LEVEL] <subcommand>
```

| Flag | Default | Description |
|---|---|---|
| `--config PATH` | `configs/default.yaml` (if present) | YAML config file to load |
| `--set KEY=VAL` | — | Dotted override, repeatable: e.g. `--set slam.match_beams=60` |
| `--log-level LEVEL` | value in config | Shorthand for `--set logging.level=LEVEL` |
| `--version` | — | Print `amr-stack 0.1.0` and exit |

### `amr slam`

Run a SLAM mapping mission and save the resulting map.

```
amr slam --mission PATH --out STEM [--max-time SECONDS]
```

| Argument | Default | Description |
|---|---|---|
| `--mission PATH` | required | Mission YAML file with `waypoints` list |
| `--out STEM` | required | Output map stem (produces `STEM.pgm` and `STEM.yaml`) |
| `--max-time S` | 240 | Sim-time ceiling in seconds |

Exits 0 when the mission completes before the time ceiling; exits 1 otherwise (but still saves
the map). Prints the final pose error vs ground truth.

### `amr nav`

Localize on a static map and navigate a sequence of goals.

```
amr nav --map YAML --goal X,Y [--goal X,Y ...] [--max-time SECONDS]
```

| Argument | Default | Description |
|---|---|---|
| `--map YAML` | required | Map YAML file (produced by `amr slam --out`) |
| `--goal X,Y` | required | Goal coordinate in world meters, repeatable |
| `--max-time S` | 180 | Sim-time budget per goal |

Goals are executed in sequence. Exits 0 iff every goal reaches `SUCCEEDED`.

### `amr demo`

Full pipeline: SLAM mapping mission → save map → navigate two goals.

```
amr demo [--out-dir DIR]
```

| Argument | Default | Description |
|---|---|---|
| `--out-dir DIR` | `maps` | Directory for the saved map files |

Prints `DEMO PASS` (exit 0) or `DEMO FAIL` (exit 1).

### `amr gui`

Launch the interactive tkinter HRI console.

```
amr gui [--map YAML]
```

| Argument | Default | Description |
|---|---|---|
| `--map YAML` | — | If given, starts in NAV mode on this map; otherwise starts in SLAM mode |

Requires a display (`DISPLAY` environment variable). On WSL2 with WSLg set `DISPLAY=:0.0`.

---

## Repository Layout

```
amr_stack/
├── pyproject.toml  setup.py  Makefile  .gitignore  README.md
├── configs/
│   ├── default.yaml                  # complete reference config (mirrors dataclass defaults)
│   ├── worlds/office.yaml            # 12 × 9 m two-room office world
│   └── missions/office_mapping.yaml  # waypoint mission used by demo and e2e test
├── maps/                             # generated maps land here (gitignored)
├── scripts/setup.sh
├── deploy/Dockerfile  docker-compose.yaml  amr.service  install.sh
├── docs/architecture.md  configuration.md  user_guide.md  plans/
├── amr/
│   ├── __init__.py              # __version__ = "0.1.0"
│   ├── cli.py
│   ├── core/    types.py  geometry.py  config.py  log.py
│   ├── sim/     world.py  robot.py  lidar.py  simulator.py
│   ├── mapping/ map_io.py  occupancy_grid_mapper.py
│   ├── slam/    scan_matching_slam.py
│   ├── localization/ motion_model.py  sensor_model.py  mcl.py
│   ├── planning/ costmap.py  astar.py  dwa.py
│   ├── navigation/ navigator.py
│   ├── runtime/ app.py
│   └── gui/     app.py  map_canvas.py  panels.py
└── tests/  (17 test files, 43 assertions)
```

---

## Running Tests

```bash
make test           # full suite including slow estimation tests
make test-fast      # skip tests marked @pytest.mark.slow
```

The test runner invokes pytest inside the venv:

```bash
.venv/bin/python -m pytest -q
```

See `docs/user_guide.md` for troubleshooting, GUI walkthrough, and deployment instructions.
