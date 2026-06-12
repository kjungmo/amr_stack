# AMR Stack — User Guide

## Contents

1. [Headless CLI Recipes](#1-headless-cli-recipes)
2. [GUI Walkthrough](#2-gui-walkthrough)
3. [Map File Format](#3-map-file-format)
4. [Deployment Guide](#4-deployment-guide)
5. [Troubleshooting](#5-troubleshooting)

---

## 1. Headless CLI Recipes

All commands run inside the project venv (`.venv/bin/amr`) or via the Makefile.

### Run the full autonomy demo

```bash
make demo
# or directly:
.venv/bin/amr demo
```

The demo runs in three phases:

1. **SLAM phase:** the robot autonomously drives the office waypoint mission
   (`configs/missions/office_mapping.yaml`), building a map as it goes.
2. **Save:** the completed map is written to `maps/office.pgm` and `maps/office.yaml`.
3. **NAV phase:** on the saved map, the robot localizes with MCL and navigates to
   `(10.5, 1.5)` then `(2.0, 7.0)`.

Prints `DEMO PASS` (exit 0) or `DEMO FAIL` (exit 1). Typical wall time: under 3 minutes.

### Run only the SLAM mapping mission

```bash
.venv/bin/amr slam \
    --mission configs/missions/office_mapping.yaml \
    --out maps/mymap \
    --max-time 300
```

Saves `maps/mymap.pgm` and `maps/mymap.yaml` on completion or timeout.

### Navigate to a sequence of goals on a saved map

```bash
.venv/bin/amr nav \
    --map maps/office.yaml \
    --goal 10.5,1.5 \
    --goal 2.0,7.0 \
    --max-time 180
```

Goals are executed in sequence. Returns exit 0 iff every goal succeeds.

### Apply configuration overrides from the CLI

Any config key can be overridden with `--set dotted.path=value`:

```bash
# Speed up SLAM by using fewer beams (less accurate but faster)
.venv/bin/amr slam \
    --mission configs/missions/office_mapping.yaml \
    --out maps/fast \
    --set slam.match_beams=40

# Use more particles for better localization
.venv/bin/amr nav \
    --map maps/office.yaml \
    --goal 5.0,5.0 \
    --set localization.num_particles=1000

# Suppress console output
.venv/bin/amr demo --set logging.console=false
```

### Use a custom config file

```bash
.venv/bin/amr --config myrobot.yaml slam \
    --mission configs/missions/office_mapping.yaml \
    --out maps/myrobot
```

The YAML file may set any subset of keys; unset keys take their dataclass defaults.
See `docs/configuration.md` for the full table.

---

## 2. GUI Walkthrough

Launch the GUI with:

```bash
make gui
# or:
.venv/bin/amr gui                      # SLAM mode (no map)
.venv/bin/amr gui --map maps/office.yaml   # NAV mode on a saved map
```

A display is required (`DISPLAY` environment variable). On WSL2 with WSLg, set
`DISPLAY=:0.0` in your shell profile if the variable is not already set.

### Main Window Layout

```
┌─────────────────────────────────────────────────────────┐
│  [SLAM Mapping] [Navigate]    [Save Map] [E-STOP] [Resume] [Manual ✓]  Speed: ——●—— │
├─────────────────────────────────────────────────────────┤
│                                                           │
│                  Map Canvas                               │
│        (occupancy grid + overlays)                        │
│                                                           │
├─────────────────────────────────────────────────────────┤
│  Status: nav_state | pose x,y,θ | sim time | ESTOP | collision        │
├─────────────────────────────────────────────────────────┤
│  Log Panel (scrolling text)                              │
└─────────────────────────────────────────────────────────┘
```

### Map Canvas

The central canvas displays:

| Visual element | Description |
|---|---|
| Gray background | Unknown cells (not yet visited) |
| Light cells | Free space |
| Dark cells | Occupied cells (walls, obstacles) |
| Blue circle + line | Robot pose estimate (circle = footprint, line = heading) |
| Gray circle + line | Ground truth pose (toggleable; shown for comparison) |
| Red dots | Lidar scan endpoints (up to 120 points) |
| Green ticks | MCL particles (up to 300, shown in NAV mode) |
| Orange polyline | Current planned global path |
| Cross marker | Active navigation goal |

The map is re-rendered only when `map_version` increments (on each scan integration), so the
canvas does not flicker between map updates.

**Mouse interactions:**

| Action | Mode | Effect |
|---|---|---|
| Left-click on canvas | NAV mode | Sets navigation goal at clicked world coordinate → sends `SetGoal` command |
| Click-drag on canvas | "Set Pose" toggle active | Sets initial pose estimate: click position = x,y; drag direction = heading → sends `SetInitialPose` |

### Mode Panel

| Button | Description |
|---|---|
| `[SLAM Mapping]` | Switch to SLAM mode; starts a fresh mapping session. If currently in NAV mode, this re-initializes the SLAM system. |
| `[Navigate]` | Switch to NAV mode. If currently in SLAM mode, prompts to save the current map first via `SaveMap`, then initializes the MCL localizer on that map. |

### Actions Panel

| Control | Description |
|---|---|
| `[Save Map]` | Saves the current map (live SLAM map or loaded static map) via `SaveMap` command |
| `[E-STOP]` | Immediately halts the robot; sends `EStop`. The robot freezes and the navigator state is not advanced while e-stop is active. |
| `[Resume]` | Clears the e-stop; sends `Resume`. The robot resumes autonomous or manual operation. |
| `[Manual ✓]` | Toggle manual (teleop) mode; sends `SetManual`. In manual mode the keyboard controls the robot instead of the autonomous planner. |
| Speed scale | Slider 0.5×–4×; adjusts the worker thread pacing (`speed_factor`). At 1× the simulation runs at real time; higher values run faster than real time. |

### Status Bar

The status bar shows (left to right):

- `nav_state`: MAPPING / PLANNING / FOLLOWING / RECOVERY / SUCCEEDED / FAILED
- `pose`: estimated x, y (m) and θ (rad)
- `sim_time`: simulated time in seconds
- `ESTOP` flag: highlighted in red when e-stop is engaged
- `collision` flag: highlighted when the simulated robot has collided with an obstacle

### Log Panel

A scrolling `tk.Text` widget shows timestamped log messages from the `amr` logger. Messages are
color-coded: WARNING entries are highlighted in orange, ERROR entries in red. The panel retains
the last 500 lines. Log verbosity is controlled by `logging.level` in the config
(default `INFO`; set to `DEBUG` for verbose output).

### Keyboard Bindings (Manual Mode)

Manual mode must be active (`[Manual ✓]` toggled on) for keyboard control.

| Key | Action |
|---|---|
| `W` or `Up arrow` | Forward: `SetTeleop(v=+max_lin_vel, omega=0)` |
| `S` or `Down arrow` | Backward: `SetTeleop(v=-max_lin_vel, omega=0)` |
| `A` or `Left arrow` | Turn left: `SetTeleop(v=0, omega=+max_ang_vel)` |
| `D` or `Right arrow` | Turn right: `SetTeleop(v=0, omega=-max_ang_vel)` |

Key-release events reset the teleop command to zero, so the robot stops when you release a key.

### Threading Model

The GUI uses a strict two-thread architecture to prevent tkinter cross-thread crashes:

- **Worker thread:** owns `AmrApp`, loops `app.step()` paced to `dt / speed_factor` wall
  seconds, publishes the latest `Snapshot` into a thread-safe mailbox, and drains a command
  queue into `app.handle_command`.
- **Tk main thread:** polls the mailbox every `gui.refresh_ms` (default 66 ms ≈ 15 Hz) via
  `root.after`, reads the latest snapshot, and redraws the canvas and status bar. Only the Tk
  thread ever touches widgets.
- **Shutdown:** closing the window sends `Shutdown`, the worker joins cleanly, then
  `root.destroy()` is called.

---

## 3. Map File Format

Maps are stored as ROS `map_server`-compatible file pairs:

### `stem.pgm` (P5 binary PGM)

- **Format:** P5 (binary grayscale), one byte per pixel.
- **Row 0** of the image corresponds to the **maximum-y (north) edge** of the map (image
  convention: row 0 = top; the grid is flipped with `np.flipud` on write and read).
- **Pixel values:**

| Pixel value | Grid value | Meaning |
|---|---|---|
| 0 | 100 | Occupied |
| 254 | 0 | Free |
| 205 | -1 | Unknown |

### `stem.yaml`

```yaml
image: stem.pgm          # relative path to the PGM
resolution: 0.05         # meters per cell
origin: [x, y, 0.0]     # world coordinate of the outer corner of cell (row=0, col=0)
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

`origin` is the world position of the lower-left corner of the grid (minimum-x, minimum-y
in world coordinates after the y-flip is undone on load).

### Loading and Saving Programmatically

```python
from amr.mapping.map_io import load_map, save_map
from amr.core.types import OccupancyGrid
import numpy as np

# Save
grid = OccupancyGrid(resolution=0.05, origin_x=0.0, origin_y=0.0,
                     data=np.zeros((180, 240), dtype=np.int8))
pgm_path, yaml_path = save_map(grid, "maps/mymap")  # produces mymap.pgm + mymap.yaml

# Load
grid = load_map("maps/mymap.yaml")
```

---

## 4. Deployment Guide

### Docker

Build and run the demo:

```bash
cd /path/to/amr_stack
docker build -f deploy/Dockerfile -t amr-stack:latest .
docker run --rm amr-stack:latest demo
```

With docker-compose (demo + GUI variants):

```bash
# Headless demo
docker compose -f deploy/docker-compose.yaml run amr-demo

# GUI (requires X11 forwarding)
xhost +local:docker
docker compose -f deploy/docker-compose.yaml run amr-gui
```

**Note:** docker build and runtime were not executed on the development machine (WSL2, no docker
daemon). The Dockerfile and compose file have been statically validated with `bash -n` and
`yaml.safe_load`; runtime validation is deferred.

### systemd Service (Linux target machine)

Deploy to `/opt/amr` and register the systemd service:

```bash
sudo bash deploy/install.sh              # default destination: /opt/amr
sudo bash deploy/install.sh /srv/amr    # custom destination
```

The installer:
1. Copies the project (excluding `.venv`, `.git`, `logs`) to the destination directory.
2. Creates a new venv at `DEST/.venv` with `--system-site-packages`.
3. Installs the package.
4. Copies `deploy/amr.service` to `/etc/systemd/system/amr.service` and runs
   `systemctl daemon-reload`.

Enable and start:

```bash
sudo systemctl enable --now amr.service
sudo journalctl -u amr.service -f      # follow logs
```

The service runs `amr demo` on start. To run a different command, edit the `ExecStart` line in
`/etc/systemd/system/amr.service`.

**Note:** `systemd` runtime deployment was not tested on the development machine. Static
validation of the service file is performed via `configparser` in the CI gate.

---

## 5. Troubleshooting

### No display available (GUI fails to start)

```
_tkinter.TclError: no display name and no $DISPLAY environment variable
```

On WSL2 with WSLg, ensure the DISPLAY variable is set:

```bash
export DISPLAY=:0.0
.venv/bin/amr gui
```

On a headless server without a display, only headless commands (`slam`, `nav`, `demo`) are
available. The GUI smoke test is skipped automatically when `DISPLAY` is not set.

### SLAM is slow or the mapping mission times out

The scan-matching search scales with `match_beams · Kθ · Kxy`. Reduce the beam count:

```bash
.venv/bin/amr slam \
    --mission configs/missions/office_mapping.yaml \
    --out maps/fast \
    --set slam.match_beams=40
```

Values as low as 30–40 still produce usable maps on the office world. For large environments
with slow processing, also consider narrowing the search windows:

```bash
--set slam.coarse_window_xy=0.10 \
--set slam.coarse_step_xy=0.05
```

### Localization is flaky (robot loses track of its position)

Increase the particle count:

```bash
.venv/bin/amr nav \
    --map maps/office.yaml \
    --goal 10.5,1.5 \
    --set localization.num_particles=1000
```

Also check that the loaded map was built from a complete mapping mission covering the area where
localization is failing. Partial maps leave large unknown regions, and `unknown_is_lethal=True`
(the default) treats those regions as impassable.

If the robot consistently localizes in the wrong room, reset the initial pose estimate with
`SetInitialPose` (click-drag on the map canvas in the GUI) or run a global localization by
setting `num_particles=2000` with a uniform prior (start without an `initial_pose`).

### Broken ensurepip on Ubuntu 20.04

If `make setup` fails with:

```
Error: ensurepip is not available
```

The `scripts/setup.sh` script handles this automatically by downloading a Python 3.8-compatible
`get-pip.py` from `https://bootstrap.pypa.io/pip/3.8/get-pip.py` and bootstrapping pip into the
venv. If you are on a network-isolated machine, pre-download the file and set the environment
variable before running setup:

```bash
wget https://bootstrap.pypa.io/pip/3.8/get-pip.py -O /tmp/get-pip-3.8.py
GET_PIP_PY=/tmp/get-pip-3.8.py bash scripts/setup.sh
```

### Tests fail with import errors after adding a new module

Ensure the package is installed in editable mode inside the venv:

```bash
.venv/bin/pip install -e ".[dev]"
```

### ROS Noetic / system PYTHONPATH conflicts

The setup script unsets `PYTHONPATH` before creating the venv to prevent ROS's dist-packages
from interfering:

```bash
unset PYTHONPATH
make setup
```

If you run pytest outside the Makefile, prefix the command with `env -u PYTHONPATH`:

```bash
env -u PYTHONPATH .venv/bin/python -m pytest -q
```
