# AMR Stack — Complete Autonomous Mobile Robot Software Package: Implementation Plan

> **For agentic workers:** This plan is designed for execution by an Opus-led dynamic
> workflow (subagents per task/lane). Use `superpowers:subagent-driven-development`
> semantics: one fresh subagent per task, TDD steps tracked via the `- [ ]` checkboxes,
> review between tasks, hard verification gates between phases. Orchestration guidance
> (parallel lanes, gate commands, fix-loop) is in **Appendix A**. Loop on the final gate
> until everything is green — that is the definition of done.

**Goal:** A self-contained, pip-installable Python package (`amr-stack`) implementing a complete
autonomous mobile robot software stack — 2D simulation, occupancy-grid mapping, scan-matching
SLAM, Monte-Carlo localization, A* global + DWA local planning, a navigation state machine,
file-based configuration, structured logging, a tkinter HRI GUI, and deployment/setup artifacts —
verified end-to-end by an autonomous "SLAM-the-world, save map, localize, navigate to goals" test.

**Architecture:** Layered library with a thin runtime conductor. `amr.core` defines shared types,
geometry, config and logging (the frozen contracts). Independent subsystems (`sim`, `mapping`,
`planning`, `localization`, `slam`, `navigation`) depend only on `amr.core` and each other's public
APIs listed here. `amr.runtime.AmrApp` wires subsystems into two modes (SLAM / NAV) behind a
Snapshot/Command interface consumed identically by the headless CLI and the GUI.

**Tech Stack:** Python 3.8.10 (system), numpy 1.17.4, PyYAML, tkinter (GUI), pytest (dev, in venv).
**No ROS, no scipy** — all algorithms implemented from scratch per the specs below.

---

## 1. Environment facts (verified 2026-06-12 on the target machine)

| Fact | Value | Consequence |
|---|---|---|
| OS | WSL2 Ubuntu 20.04, WSLg `DISPLAY=:0.0` | GUI can be launched and visually checked |
| Python | 3.8.10 | **No** `match`, no `dict \| dict`, no `str.removeprefix`, no builtin-generic annotations at runtime → use `from __future__ import annotations` |
| numpy | 1.17.4 | `np.random.default_rng` available; keep to basic vectorized ops |
| PyYAML / tkinter / PyQt5 / PIL / matplotlib | present (system site-packages) | GUI = **tkinter** (stdlib-style, simplest) |
| scipy | **ABSENT** | distance transforms & blurs implemented in numpy (specs provided) |
| pytest / pip | **ABSENT** system-wide; `python3 -m venv` works | venv is created with `--system-site-packages`, then `pip install -e ".[dev]"` |
| docker | **ABSENT**; systemd not PID 1 | Dockerfile/systemd unit are authored + statically validated; runtime validation deferred (note it in the final report) |
| git | 2.25.1 | repo is `git init`-ed in Task 0.1 |

**Project root:** `/home/cona/kangj/amr_stack` (this plan lives at `docs/plans/` inside it).

## 2. Coordinate & data conventions (FROZEN — every module obeys these)

1. **World frame:** x right, y up, meters. `theta` CCW radians wrapped to `(-pi, pi]`.
2. **Robot frame:** x forward, y left. Lidar beam angle 0 = robot forward.
3. **Grid indexing:** `data[row, col]`; `col` maps to x, `row` maps to y; **row 0 = minimum-y (south) edge**. No flipping anywhere except PGM image I/O (image row 0 = top).
4. **Cell ↔ world:** `col = floor((x - origin_x)/res)`; `grid_to_world` returns **cell centers**.
5. **Occupancy values (int8, ROS convention):** `-1` unknown, `0` free, `100` occupied. A cell is "occupied" iff `data >= 65`.
6. **Map files:** ROS `map_server`-compatible pair `stem.pgm` (P5 binary: occupied→0, free→254, unknown→205, `np.flipud` on write/read) + `stem.yaml` (`image, resolution, origin: [x, y, 0], negate: 0, occupied_thresh: 0.65, free_thresh: 0.25`).
7. **Determinism:** every stochastic component takes an `np.random.Generator`; all are seeded from `cfg.seed` (default 42). Tests rely on this — never call `np.random.*` module-level functions.
8. **Units:** SI everywhere (m, rad, s, m/s, rad/s).

## 3. Repository layout

```
amr_stack/
├── pyproject.toml  setup.py  Makefile  .gitignore  README.md
├── configs/
│   ├── default.yaml                 # complete reference config (== dataclass defaults)
│   ├── worlds/office.yaml           # 12×9 m two-room office world
│   └── missions/office_mapping.yaml # waypoint mission used by SLAM demo/e2e
├── maps/.gitkeep                    # generated maps land here (gitignored)
├── scripts/setup.sh
├── deploy/Dockerfile  docker-compose.yaml  amr.service  install.sh
├── docs/architecture.md  configuration.md  user_guide.md  plans/<this file>
├── amr/
│   ├── __init__.py                  # __version__ = "0.1.0"
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
└── tests/  conftest.py  test_types.py test_geometry.py test_config.py test_log.py
            test_map_io.py test_mapper.py test_sim.py test_costmap.py test_astar.py
            test_dwa.py test_motion_sensor.py test_mcl.py test_slam.py
            test_navigator.py test_cli.py test_gui_smoke.py test_integration_e2e.py
```

Every `amr/*/` directory gets an `__init__.py` re-exporting its public names.

## 4. Capability → component map (the 10 required capabilities)

| Required capability | Delivered by | Proven by |
|---|---|---|
| Mapping | `amr.mapping` (log-odds grid mapper + ROS-format map I/O) | `test_mapper.py`, `test_map_io.py` |
| Configuration via file | `amr.core.config` + `configs/*.yaml` (+ `--set` overrides) | `test_config.py` |
| Logging | `amr.core.log` (console + rotating file + per-module levels + GUI handler) | `test_log.py` |
| Planning | `amr.planning` (costmap + A* global + DWA local) | `test_costmap/astar/dwa.py` |
| Localization | `amr.localization` (MCL particle filter, likelihood field) | `test_mcl.py` |
| SLAM | `amr.slam` (correlative scan-matching + log-odds map) | `test_slam.py` |
| Simulation | `amr.sim` (diff-drive, vectorized lidar, odom noise, collisions) | `test_sim.py` |
| Deployment | `deploy/` (Dockerfile, compose, systemd unit, install.sh) | Task 4.2 static checks |
| Setups | `scripts/setup.sh`, `Makefile`, `pyproject.toml`, README quickstart | Gate A |
| GUI for HRI | `amr.gui` (tkinter console: map/robot/particles/path, click-to-goal, teleop, e-stop, log panel) | `test_gui_smoke.py` + WSLg visual check |

---

# Phase 0 — Foundations (SERIAL; everything depends on this)

## Task 0.1: Repository scaffold, packaging, venv, CI-able test runner

**Files:** Create `pyproject.toml`, `setup.py`, `.gitignore`, `Makefile`, `scripts/setup.sh`,
`README.md` (stub), `amr/__init__.py`, `amr/cli.py` (stub), all package `__init__.py` files,
`maps/.gitkeep`, `tests/test_scaffold.py`.

- [ ] **Step 1: git init + author files.**

`pyproject.toml`:
```toml
[build-system]
requires = ["setuptools>=44"]
build-backend = "setuptools.build_meta"

[project]
name = "amr-stack"
version = "0.1.0"
description = "Self-contained AMR software stack: sim, mapping, SLAM, MCL, planning, nav, HRI GUI"
requires-python = ">=3.8"
dependencies = ["numpy>=1.17", "PyYAML>=5.3"]

[project.optional-dependencies]
dev = ["pytest>=7"]

[project.scripts]
amr = "amr.cli:main"

[tool.setuptools.packages.find]
include = ["amr*"]

[tool.pytest.ini_options]
testpaths = ["tests"]
markers = ["slow: long-running estimation/integration tests"]
```

`setup.py` (compat shim so editable installs work on any pip ≥ 20):
```python
from setuptools import setup

setup()
```

`scripts/setup.sh`:
```bash
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv --system-site-packages .venv
.venv/bin/pip install --upgrade pip setuptools wheel
.venv/bin/pip install -e ".[dev]"
.venv/bin/python -c "import amr, numpy, yaml; print('amr', amr.__version__, 'ready')"
```

`Makefile`:
```makefile
PY  := .venv/bin/python
AMR := .venv/bin/amr

.PHONY: setup test test-fast demo gui clean

setup:
	bash scripts/setup.sh
test:
	$(PY) -m pytest -q
test-fast:
	$(PY) -m pytest -q -m "not slow"
demo:
	$(AMR) demo
gui:
	$(AMR) gui
clean:
	rm -rf .venv build dist *.egg-info logs $$(find . -name __pycache__)
```

`.gitignore`: `.venv/`, `__pycache__/`, `*.pyc`, `*.egg-info/`, `build/`, `dist/`,
`logs/`, `maps/*.pgm`, `maps/*.yaml` (keep `maps/.gitkeep` via `!maps/.gitkeep`).

`amr/__init__.py`: `__version__ = "0.1.0"`.

`amr/cli.py` stub (replaced in Task 3.1, but the console script must work from day one):
```python
import argparse


def main(argv=None):
    parser = argparse.ArgumentParser(prog="amr", description="AMR stack CLI")
    parser.add_argument("--version", action="version", version="amr-stack 0.1.0")
    parser.parse_args(argv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`tests/test_scaffold.py`:
```python
import amr


def test_version():
    assert amr.__version__ == "0.1.0"
```

- [ ] **Step 2:** Run `git init -b main && bash scripts/setup.sh`. Expected: ends with `amr 0.1.0 ready`.
- [ ] **Step 3:** Run `.venv/bin/python -m pytest -q`. Expected: `1 passed`.
- [ ] **Step 4:** Run `.venv/bin/amr --version`. Expected: `amr-stack 0.1.0`.
- [ ] **Step 5:** Commit: `git add -A && git commit -m "chore: scaffold amr-stack package, venv setup, test runner"`.

## Task 0.2: Core types + geometry (`amr/core/types.py`, `amr/core/geometry.py`)

**Files:** Create `amr/core/types.py`, `amr/core/geometry.py`, `tests/test_types.py`, `tests/test_geometry.py`.

- [ ] **Step 1: Write the failing tests.**

`tests/test_geometry.py`:
```python
import math

import numpy as np
import pytest

from amr.core.geometry import (bresenham, distance_field, pose_between,
                               pose_compose, transform_points, wrap_angle)
from amr.core.types import Pose2D


def test_wrap_angle():
    assert wrap_angle(0.0) == 0.0
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)      # (-pi, pi]
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3.5 * math.pi) == pytest.approx(-0.5 * math.pi)


def test_compose_between_roundtrip():
    a = Pose2D(1.0, 2.0, 0.7)
    b = Pose2D(-0.5, 3.0, -2.0)
    d = pose_between(a, b)
    c = pose_compose(a, d)
    assert (c.x, c.y) == pytest.approx((b.x, b.y), abs=1e-9)
    assert wrap_angle(c.theta - b.theta) == pytest.approx(0.0, abs=1e-9)


def test_transform_points():
    pts = np.array([[1.0, 0.0], [0.0, 1.0]])
    out = transform_points(Pose2D(2.0, 1.0, math.pi / 2), pts)
    assert out[0] == pytest.approx([2.0, 2.0])   # forward point ends up +y
    assert out[1] == pytest.approx([1.0, 1.0])


def test_bresenham():
    assert bresenham(0, 0, 0, 3) == [(0, 0), (0, 1), (0, 2), (0, 3)]
    assert bresenham(0, 0, 3, 3) == [(0, 0), (1, 1), (2, 2), (3, 3)]
    cells = bresenham(0, 0, 1, 3)
    assert cells[0] == (0, 0) and cells[-1] == (1, 3) and len(cells) == 4
    assert bresenham(2, 5, 2, 5) == [(2, 5)]


def test_distance_field():
    occ = np.zeros((11, 11), dtype=bool)
    occ[5, 5] = True
    d = distance_field(occ, resolution=0.1, max_dist=0.5)
    assert d[5, 5] == 0.0
    assert d[5, 8] == pytest.approx(0.3, abs=0.01)        # 3 cells straight
    assert d[8, 8] == pytest.approx(0.3 * math.sqrt(2), abs=0.05)
    assert d[0, 0] == pytest.approx(0.5)                  # capped at max_dist
```

`tests/test_types.py`:
```python
import numpy as np
import pytest

from amr.core.types import LaserScan, OccupancyGrid


def test_grid_world_roundtrip():
    g = OccupancyGrid(resolution=0.5, origin_x=-1.0, origin_y=2.0,
                      data=np.zeros((4, 6), dtype=np.int8))
    assert g.rows == 4 and g.cols == 6
    r, c = g.world_to_grid(0.6, 3.2)
    assert (r, c) == (2, 3)
    x, y = g.grid_to_world(2, 3)
    assert (x, y) == pytest.approx((0.75, 3.25))          # cell center
    assert g.world_to_grid(x, y) == (2, 3)
    assert g.in_bounds(0, 0) and not g.in_bounds(4, 0) and not g.in_bounds(-1, 2)


def test_scan_helpers():
    s = LaserScan(angle_min=-1.0, angle_increment=0.5, range_min=0.1,
                  range_max=5.0, ranges=np.array([0.05, 2.0, 5.0, 4.9]))
    assert s.num_beams == 4
    assert s.angles() == pytest.approx([-1.0, -0.5, 0.0, 0.5])
    assert list(s.valid_mask()) == [False, True, False, True]
```

- [ ] **Step 2:** `.venv/bin/python -m pytest tests/test_types.py tests/test_geometry.py -q` — expected: collection errors (modules missing).
- [ ] **Step 3: Implement.**

`amr/core/types.py` (complete):
```python
"""Shared AMR data types. See plan §2 for the frozen conventions."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # rad, CCW, wrapped to (-pi, pi]

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)

    @staticmethod
    def from_array(a) -> "Pose2D":
        return Pose2D(float(a[0]), float(a[1]), float(a[2]))


@dataclass
class Twist2D:
    v: float = 0.0      # m/s forward
    omega: float = 0.0  # rad/s CCW


@dataclass
class LaserScan:
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float            # no-return is encoded as range_max
    ranges: np.ndarray          # (N,) float
    stamp: float = 0.0          # sim time, s

    @property
    def num_beams(self) -> int:
        return int(self.ranges.shape[0])

    def angles(self) -> np.ndarray:
        return self.angle_min + self.angle_increment * np.arange(self.num_beams)

    def valid_mask(self) -> np.ndarray:
        return (self.ranges > self.range_min) & (self.ranges < self.range_max * 0.999)


@dataclass
class OccupancyGrid:
    resolution: float   # m/cell
    origin_x: float     # world x of the outer corner of cell (row=0, col=0)
    origin_y: float
    data: np.ndarray    # int8 (rows, cols): -1 unknown, 0 free, 100 occupied

    @property
    def rows(self) -> int:
        return int(self.data.shape[0])

    @property
    def cols(self) -> int:
        return int(self.data.shape[1])

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((y - self.origin_y) / self.resolution)),
                int(math.floor((x - self.origin_x) / self.resolution)))

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    def occupied_mask(self, thresh: int = 65) -> np.ndarray:
        return self.data >= thresh

    def copy(self) -> "OccupancyGrid":
        return OccupancyGrid(self.resolution, self.origin_x, self.origin_y,
                             self.data.copy())
```

`amr/core/geometry.py` (complete):
```python
"""Geometry primitives shared by all subsystems."""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

from amr.core.types import Pose2D


def wrap_angle(a: float) -> float:
    """Wrap a scalar angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def wrap_angles(a: np.ndarray) -> np.ndarray:
    """Vector wrap to [-pi, pi) — note the half-open end differs from wrap_angle."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def pose_compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """a ⊕ b: pose of frame b (expressed in a) in a's parent frame."""
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose2D(a.x + c * b.x - s * b.y,
                  a.y + s * b.x + c * b.y,
                  wrap_angle(a.theta + b.theta))


def pose_between(a: Pose2D, b: Pose2D) -> Pose2D:
    """a ⊖ b: pose of b expressed in frame a, so pose_compose(a, result) == b."""
    dx, dy = b.x - a.x, b.y - a.y
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose2D(c * dx + s * dy, -s * dx + c * dy, wrap_angle(b.theta - a.theta))


def transform_points(pose: Pose2D, pts: np.ndarray) -> np.ndarray:
    """Transform (N, 2) points from pose's frame into the world frame."""
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T + np.array([pose.x, pose.y])


def bresenham(r0: int, c0: int, r1: int, c1: int) -> List[Tuple[int, int]]:
    """All integer grid cells on the line from (r0,c0) to (r1,c1), inclusive."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr
    return cells


def distance_field(occupied: np.ndarray, resolution: float,
                   max_dist: float) -> np.ndarray:
    """Chamfer distance (m) from each cell to the nearest occupied cell.

    Iterative 8-neighbour relaxation (Bellman-Ford style), fully vectorized;
    converges in <= ceil(max_dist/resolution) sweeps. No scipy required.
    """
    big = max_dist / resolution + 2.0
    d = np.where(occupied, 0.0, big)
    sq2 = math.sqrt(2.0)
    for _ in range(int(math.ceil(max_dist / resolution)) + 1):
        nd = d.copy()
        nd[1:, :] = np.minimum(nd[1:, :], d[:-1, :] + 1.0)
        nd[:-1, :] = np.minimum(nd[:-1, :], d[1:, :] + 1.0)
        nd[:, 1:] = np.minimum(nd[:, 1:], d[:, :-1] + 1.0)
        nd[:, :-1] = np.minimum(nd[:, :-1], d[:, 1:] + 1.0)
        nd[1:, 1:] = np.minimum(nd[1:, 1:], d[:-1, :-1] + sq2)
        nd[1:, :-1] = np.minimum(nd[1:, :-1], d[:-1, 1:] + sq2)
        nd[:-1, 1:] = np.minimum(nd[:-1, 1:], d[1:, :-1] + sq2)
        nd[:-1, :-1] = np.minimum(nd[:-1, :-1], d[1:, 1:] + sq2)
        if np.array_equal(nd, d):
            break
        d = nd
    return np.minimum(d * resolution, max_dist).astype(np.float32)
```

- [ ] **Step 4:** `.venv/bin/python -m pytest tests/test_types.py tests/test_geometry.py -q` — expected: all pass.
- [ ] **Step 5:** Commit: `git commit -am "feat(core): shared types and geometry primitives"`.

## Task 0.3: Configuration system (`amr/core/config.py` + `configs/default.yaml`)

**Files:** Create `amr/core/config.py`, `configs/default.yaml`, `tests/test_config.py`.

- [ ] **Step 1: Write the failing tests** (`tests/test_config.py`):
```python
import dataclasses

import pytest

from amr.core.config import AmrConfig, ConfigError, apply_overrides, load_config


def test_defaults_match_reference_yaml():
    # configs/default.yaml is the complete documented reference; it must stay
    # in lockstep with the dataclass defaults.
    assert load_config("configs/default.yaml") == AmrConfig()


def test_partial_yaml_and_overrides(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("robot:\n  radius: 0.25\nlocalization:\n  num_particles: 100\n")
    cfg = load_config(str(p))
    assert cfg.robot.radius == 0.25
    assert cfg.localization.num_particles == 100
    assert cfg.nav.goal_tol_xy == AmrConfig().nav.goal_tol_xy  # untouched default

    cfg = apply_overrides(cfg, ["nav.goal_tol_xy=0.5", "slam.match_beams=40"])
    assert cfg.nav.goal_tol_xy == 0.5 and cfg.slam.match_beams == 40


def test_unknown_key_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("robot:\n  radíus_typo: 0.2\n")
    with pytest.raises(ConfigError):
        load_config(str(p))
    with pytest.raises(ConfigError):
        apply_overrides(AmrConfig(), ["robot.nope=1"])


def test_type_mismatch_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("localization:\n  num_particles: many\n")
    with pytest.raises(ConfigError):
        load_config(str(p))


def test_config_is_dataclass_tree():
    cfg = AmrConfig()
    assert dataclasses.is_dataclass(cfg.planning.dwa)
```

- [ ] **Step 2:** Run them — expected: import errors.
- [ ] **Step 3: Implement** `amr/core/config.py` (complete — this is a frozen contract; every
  field name below is used verbatim by later tasks):
```python
"""File-based configuration: dataclass schema + YAML loader + dotted overrides.

Dataclass defaults are canonical. YAML files may set any subset of keys;
unknown keys and type mismatches raise ConfigError with a dotted path.
"""
import copy
import dataclasses
import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, get_args, get_origin, get_type_hints

import yaml


class ConfigError(Exception):
    pass


@dataclass
class RobotConfig:
    radius: float = 0.18
    max_lin_vel: float = 0.6
    max_ang_vel: float = 1.8
    max_lin_acc: float = 0.8
    max_ang_acc: float = 2.5


@dataclass
class LidarConfig:
    num_beams: int = 240
    angle_min: float = -math.pi
    angle_max: float = math.pi          # increment = (max - min) / num_beams
    range_min: float = 0.12
    range_max: float = 8.0
    noise_std: float = 0.01
    scan_every: int = 2                 # emit a scan every N sim steps


@dataclass
class OdomNoiseConfig:
    alpha_v: float = 0.03               # std(v_meas) = alpha_v*|v| + floor
    alpha_w: float = 0.03
    floor: float = 1e-4


@dataclass
class SimConfig:
    dt: float = 0.05
    world_file: str = "configs/worlds/office.yaml"
    odom_noise: OdomNoiseConfig = field(default_factory=OdomNoiseConfig)


@dataclass
class MappingConfig:
    resolution: float = 0.05
    l_occ: float = 0.85
    l_free: float = -0.4
    l_clamp: float = 10.0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.25
    beam_subsample: int = 2


@dataclass
class SlamConfig:
    keyframe_trans: float = 0.2         # integrate scan into map after this motion
    keyframe_rot: float = 0.35
    min_motion: float = 0.02            # skip matching below this displacement
    match_beams: int = 80
    coarse_window_xy: float = 0.15
    coarse_step_xy: float = 0.05
    coarse_window_theta: float = 0.12
    coarse_step_theta: float = 0.03
    fine_step_xy: float = 0.025
    fine_step_theta: float = 0.01
    blur_sigma_cells: float = 1.5
    min_match_score: float = 0.1


@dataclass
class LikelihoodConfig:
    sigma_hit: float = 0.2
    z_hit: float = 0.9
    z_rand: float = 0.1
    max_dist: float = 2.0
    beam_subsample: int = 5


@dataclass
class LocalizationConfig:
    num_particles: int = 500
    alphas: List[float] = field(default_factory=lambda: [0.05, 0.05, 0.05, 0.05])
    init_std: List[float] = field(default_factory=lambda: [0.25, 0.25, 0.15])
    resample_neff_frac: float = 0.5
    likelihood: LikelihoodConfig = field(default_factory=LikelihoodConfig)


@dataclass
class CostmapConfig:
    occupied_thresh: int = 65
    unknown_is_lethal: bool = True
    inflation_radius: float = 0.45
    cost_decay: float = 6.0


@dataclass
class AstarConfig:
    w_cost: float = 4.0
    simplify: bool = True


@dataclass
class DwaConfig:
    sim_time: float = 1.5
    sim_dt: float = 0.1
    v_samples: int = 8
    w_samples: int = 15
    lookahead: float = 0.8
    w_progress: float = 1.0
    w_heading: float = 0.6
    w_clearance: float = 0.4
    w_velocity: float = 0.3


@dataclass
class PlanningConfig:
    costmap: CostmapConfig = field(default_factory=CostmapConfig)
    astar: AstarConfig = field(default_factory=AstarConfig)
    dwa: DwaConfig = field(default_factory=DwaConfig)


@dataclass
class NavConfig:
    goal_tol_xy: float = 0.25
    replan_period: float = 4.0
    path_block_check_dist: float = 1.0
    max_recoveries: int = 3
    recovery_rotate_speed: float = 0.8
    recovery_backup_dist: float = 0.3
    recovery_backup_speed: float = 0.1


@dataclass
class LoggingConfig:
    level: str = "INFO"
    file: str = "logs/amr.log"
    max_bytes: int = 1000000
    backup_count: int = 3
    console: bool = True
    module_levels: Dict[str, str] = field(default_factory=dict)


@dataclass
class GuiConfig:
    refresh_ms: int = 66
    px_per_cell: int = 3
    speed_factor: float = 1.0           # worker pacing; 0 = run flat out


@dataclass
class AmrConfig:
    seed: int = 42
    robot: RobotConfig = field(default_factory=RobotConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    slam: SlamConfig = field(default_factory=SlamConfig)
    localization: LocalizationConfig = field(default_factory=LocalizationConfig)
    planning: PlanningConfig = field(default_factory=PlanningConfig)
    nav: NavConfig = field(default_factory=NavConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    gui: GuiConfig = field(default_factory=GuiConfig)


def _coerce(value, ftype, path):
    origin = get_origin(ftype)
    if dataclasses.is_dataclass(ftype):
        if not isinstance(value, dict):
            raise ConfigError("%s: expected a mapping" % path)
        return _from_dict(ftype, value, path)
    if origin is list:
        (etype,) = get_args(ftype)
        if not isinstance(value, list):
            raise ConfigError("%s: expected a list" % path)
        return [_coerce(v, etype, "%s[%d]" % (path, i)) for i, v in enumerate(value)]
    if origin is dict:
        if not isinstance(value, dict):
            raise ConfigError("%s: expected a mapping" % path)
        return dict(value)
    if ftype is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError("%s: expected a number, got %r" % (path, value))
        return float(value)
    if ftype is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError("%s: expected an int, got %r" % (path, value))
        return value
    if ftype is bool:
        if not isinstance(value, bool):
            raise ConfigError("%s: expected a bool, got %r" % (path, value))
        return value
    if ftype is str:
        if not isinstance(value, str):
            raise ConfigError("%s: expected a string, got %r" % (path, value))
        return value
    return value


def _from_dict(dc_type, d, path=""):
    hints = get_type_hints(dc_type)
    names = {f.name for f in dataclasses.fields(dc_type)}
    unknown = set(d) - names
    if unknown:
        raise ConfigError("unknown config key(s) %s under '%s'"
                          % (sorted(unknown), path or "root"))
    kwargs = {k: _coerce(v, hints[k], (path + "." + k).lstrip("."))
              for k, v in d.items()}
    return dc_type(**kwargs)


def load_config(path=None, overrides: Sequence[str] = ()) -> AmrConfig:
    if path is None:
        cfg = AmrConfig()
    else:
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        cfg = _from_dict(AmrConfig, raw)
    return apply_overrides(cfg, overrides) if overrides else cfg


def apply_overrides(cfg: AmrConfig, overrides: Sequence[str]) -> AmrConfig:
    cfg = copy.deepcopy(cfg)
    for item in overrides:
        if "=" not in item:
            raise ConfigError("override '%s' must look like a.b.c=value" % item)
        dotted, _, raw = item.partition("=")
        keys = dotted.strip().split(".")
        node = cfg
        for k in keys[:-1]:
            if not hasattr(node, k):
                raise ConfigError("unknown config path '%s'" % dotted)
            node = getattr(node, k)
        leaf = keys[-1]
        if not dataclasses.is_dataclass(node) or not hasattr(node, leaf):
            raise ConfigError("unknown config path '%s'" % dotted)
        ftype = get_type_hints(type(node))[leaf]
        setattr(node, leaf, _coerce(yaml.safe_load(raw), ftype, dotted))
    return cfg
```

`configs/default.yaml` — write the **complete** mirror of every default above (same nesting,
same values; `angle_min: -3.141592653589793`, `angle_max: 3.141592653589793`,
`module_levels: {}`). The equality test in Step 1 enforces exactness.

- [ ] **Step 4:** `.venv/bin/python -m pytest tests/test_config.py -q` — all pass.
- [ ] **Step 5:** Commit: `git commit -am "feat(core): file-based config with validation and dotted overrides"`.

## Task 0.4: Logging (`amr/core/log.py`)

**Files:** Create `amr/core/log.py`, `tests/test_log.py`.

- [ ] **Step 1: Failing tests** (`tests/test_log.py`):
```python
import logging

from amr.core.config import LoggingConfig
from amr.core.log import get_logger, setup_logging


def _read(p):
    return p.read_text() if p.exists() else ""


def test_file_and_levels(tmp_path):
    cfg = LoggingConfig(level="INFO", file=str(tmp_path / "amr.log"),
                        console=False, module_levels={"amr.slam": "DEBUG"})
    setup_logging(cfg)
    get_logger("slam").debug("slam-debug-visible")
    get_logger("nav").debug("nav-debug-hidden")
    get_logger("nav").info("nav-info-visible")
    for h in logging.getLogger("amr").handlers:
        h.flush()
    text = _read(tmp_path / "amr.log")
    assert "slam-debug-visible" in text
    assert "nav-debug-hidden" not in text
    assert "nav-info-visible" in text


def test_idempotent_setup(tmp_path):
    cfg = LoggingConfig(file=str(tmp_path / "a.log"), console=False)
    setup_logging(cfg)
    setup_logging(cfg)
    assert len(logging.getLogger("amr").handlers) == 1
```

- [ ] **Step 2:** Run — fails (module missing).
- [ ] **Step 3: Implement** `amr/core/log.py` (complete):
```python
"""Logging for the AMR stack: console + rotating file, per-module levels.

The 'amr' logger is the package root; cfg.level sets its level, so
module_levels overrides (e.g. {'amr.slam': 'DEBUG'}) act relative to it.
Handlers carry no level of their own. GUI attaches its own handler via
attach_handler().
"""
import logging
import os
from logging.handlers import RotatingFileHandler

_FORMAT = "%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s"
_DATEFMT = "%H:%M:%S"


def setup_logging(cfg) -> logging.Logger:
    root = logging.getLogger("amr")
    root.setLevel(getattr(logging, cfg.level.upper()))
    root.propagate = False
    for h in list(root.handlers):
        root.removeHandler(h)
        h.close()
    fmt = logging.Formatter(_FORMAT, _DATEFMT)
    if cfg.console:
        sh = logging.StreamHandler()
        sh.setFormatter(fmt)
        root.addHandler(sh)
    if cfg.file:
        os.makedirs(os.path.dirname(cfg.file) or ".", exist_ok=True)
        fh = RotatingFileHandler(cfg.file, maxBytes=cfg.max_bytes,
                                 backupCount=cfg.backup_count)
        fh.setFormatter(fmt)
        root.addHandler(fh)
    for name, level in cfg.module_levels.items():
        logging.getLogger(name).setLevel(getattr(logging, level.upper()))
    return root


def get_logger(name: str) -> logging.Logger:
    full = name if name == "amr" or name.startswith("amr.") else "amr." + name
    return logging.getLogger(full)


def attach_handler(handler: logging.Handler) -> None:
    logging.getLogger("amr").addHandler(handler)
```

- [ ] **Step 4:** Tests pass. **Step 5:** Commit `feat(core): logging with rotating file and per-module levels`.

### GATE A (end of Phase 0)
`bash scripts/setup.sh && .venv/bin/python -m pytest -q && .venv/bin/amr --version`
→ all tests pass, console script works. **Do not start Phase 1 lanes before Gate A is green.**

---

# Phase 1 — Independent subsystems (FOUR PARALLEL LANES; each depends only on Phase 0)

Lanes touch disjoint files. Each lane commits only its own files.

## Task 1.1 (Lane SIM): Simulation — `amr/sim/{world,robot,lidar,simulator}.py`

**Files:** Create the four modules, `configs/worlds/office.yaml`, `tests/test_sim.py`,
shared fixtures in `tests/conftest.py`.

**Public API (frozen):**
```python
class World:
    grid: OccupancyGrid          # ground truth: 0 free, 100 occupied (never -1)
    spawn: Pose2D
    size: Tuple[float, float]
    @staticmethod
    def from_yaml(path: str) -> "World": ...
    @staticmethod
    def from_dict(d: dict) -> "World": ...
    def is_occupied_world(self, x: float, y: float) -> bool: ...  # out of bounds = occupied

class DiffDriveRobot:
    pose: Pose2D                 # ground truth
    vel: Twist2D                 # actual velocity after accel clamping
    collided: bool               # latched true on the step a collision blocked motion
    def __init__(self, cfg: RobotConfig, spawn: Pose2D): ...
    def step(self, cmd: Twist2D, dt: float, world: World) -> None: ...

class Lidar:
    def __init__(self, cfg: LidarConfig, rng: np.random.Generator): ...
    def scan(self, world: World, pose: Pose2D, stamp: float) -> LaserScan: ...

@dataclass
class SimStepResult:
    ground_truth: Pose2D
    odom_pose: Pose2D            # drifts from ground truth
    odom_delta: Pose2D           # this step's noisy increment, in the robot frame
    scan: Optional[LaserScan]    # every cfg.lidar.scan_every steps, else None
    collided: bool
    sim_time: float

class Simulator:
    time: float
    robot: DiffDriveRobot
    world: World
    def __init__(self, world: World, cfg: AmrConfig, rng: np.random.Generator): ...
    def step(self, cmd: Twist2D) -> SimStepResult: ...
```

**Implementation specification:**

1. **World rasterization** (`from_dict`): grid `rows = round(size_y/res)`, `cols = round(size_x/res)`,
   origin (0, 0), all `0`. Obstacles list entries:
   `{type: rect, x, y, w, h}` → occupy cells whose centers fall in `[x, x+w) × [y, y+h)`;
   `{type: circle, x, y, r}` → cell centers with distance ≤ r. Vectorize with
   `np.meshgrid` of cell-center coordinates. `spawn: {x, y, theta}`.
2. **Kinematics** (`DiffDriveRobot.step`): clamp `cmd` to velocity limits, then clamp the
   change from current `vel` by `max_*_acc * dt` (symmetric). Integrate the exact arc:
   if `|omega| < 1e-9`: straight; else `R = v/omega`,
   `x += R*(sin(th + om*dt) - sin(th))`, `y += -R*(cos(th + om*dt) - cos(th))`,
   `th = wrap_angle(th + om*dt)`.
3. **Collision**: at the candidate pose, test the center plus 8 points on the footprint
   circle (radius `cfg.radius`) with `world.is_occupied_world`. If any hit: keep the old
   pose, zero `vel`, set `collided = True` for this step (else `False`).
4. **Lidar (vectorized — a per-beam Python loop is too slow):** sample distances
   `S = np.arange(range_min, range_max, res*0.5)`; build the `(B, len(S))` grid of sample
   points via broadcasting from beam angles; convert all to integer grid indices at once;
   out-of-bounds counts as a hit. `hit = occupancy[idx] (B, S) bool`;
   `first = np.argmax(hit, axis=1)`; beams with no `True` → `range_max`; otherwise
   `range = S[first] + rng.normal(0, noise_std, size=B)`, clipped to
   `[range_min, range_max]`. Beam angles: `angle_min + (angle_max-angle_min)/num_beams * arange(B)`.
5. **Odometry noise:** measured `v' = v + rng.normal(0, alpha_v*|v| + floor)`, same for
   `omega'`. `odom_delta` = exact-arc integration of `(v', omega')` over `dt` expressed in
   the robot frame (i.e. `Pose2D(dx_forward, dy_left, dtheta)`);
   `odom_pose = pose_compose(odom_pose, odom_delta)`. `odom_pose` starts equal to spawn.

`configs/worlds/office.yaml` (complete — geometry vetted against the mission in Task 3.1):
```yaml
size: [12.0, 9.0]
resolution: 0.05
spawn: {x: 1.5, y: 1.5, theta: 0.0}
obstacles:
  - {type: rect, x: 0.0,   y: 0.0,  w: 12.0, h: 0.15}   # south wall
  - {type: rect, x: 0.0,   y: 8.85, w: 12.0, h: 0.15}   # north wall
  - {type: rect, x: 0.0,   y: 0.0,  w: 0.15, h: 9.0}    # west wall
  - {type: rect, x: 11.85, y: 0.0,  w: 0.15, h: 9.0}    # east wall
  - {type: rect, x: 4.0,   y: 0.0,  w: 0.15, h: 5.5}    # wall A (door gap y 5.5–7.0)
  - {type: rect, x: 4.0,   y: 7.0,  w: 0.15, h: 1.85}   # wall A upper
  - {type: rect, x: 8.0,   y: 3.5,  w: 0.15, h: 5.35}   # wall B (gap south, y < 3.5)
  - {type: rect, x: 5.3,   y: 6.0,  w: 1.4,  h: 0.8}    # table
  - {type: circle, x: 3.3, y: 3.5,  r: 0.3}             # pillar west room
  - {type: circle, x: 9.8, y: 6.5,  r: 0.3}             # pillar east room
```

`tests/conftest.py`:
```python
import numpy as np
import pytest

from amr.core.config import load_config
from amr.sim.world import World

BOX_WORLD = {
    "size": [6.0, 6.0], "resolution": 0.05,
    "spawn": {"x": 1.0, "y": 1.0, "theta": 0.0},
    "obstacles": [
        {"type": "rect", "x": 0.0, "y": 0.0, "w": 6.0, "h": 0.1},
        {"type": "rect", "x": 0.0, "y": 5.9, "w": 6.0, "h": 0.1},
        {"type": "rect", "x": 0.0, "y": 0.0, "w": 0.1, "h": 6.0},
        {"type": "rect", "x": 5.9, "y": 0.0, "w": 0.1, "h": 6.0},
        {"type": "rect", "x": 2.7, "y": 2.7, "w": 0.6, "h": 0.6},
    ],
}


@pytest.fixture
def box_world():
    return World.from_dict(BOX_WORLD)


@pytest.fixture
def cfg():
    c = load_config()
    c.lidar.noise_std = 0.0
    c.localization.num_particles = 300
    return c


@pytest.fixture
def rng():
    return np.random.default_rng(42)
```

- [ ] **Step 1: Failing tests** (`tests/test_sim.py`):
```python
import math

import numpy as np
import pytest

from amr.core.config import RobotConfig
from amr.core.types import Pose2D, Twist2D
from amr.sim.lidar import Lidar
from amr.sim.robot import DiffDriveRobot
from amr.sim.simulator import Simulator


def test_world_raster(box_world):
    assert box_world.grid.data.shape == (120, 120)
    assert box_world.is_occupied_world(3.0, 3.0)          # center box
    assert not box_world.is_occupied_world(1.0, 1.0)
    assert box_world.is_occupied_world(-1.0, 3.0)         # out of bounds


def test_drive_straight(box_world, cfg):
    r = DiffDriveRobot(cfg.robot, Pose2D(1.0, 1.0, 0.0))
    for _ in range(40):                                   # 2.0 s
        r.step(Twist2D(0.5, 0.0), 0.05, box_world)
    # ramp-limited distance ≈ 0.5*2 - 0.5²/(2*0.8) = 0.844
    assert r.pose.x == pytest.approx(1.844, abs=0.03)
    assert r.pose.y == pytest.approx(1.0, abs=1e-6)


def test_arc_curvature(box_world):
    fast = RobotConfig(max_lin_acc=50.0, max_ang_acc=50.0)  # no ramp: exact circle
    r = DiffDriveRobot(fast, Pose2D(3.0, 1.0, 0.0))
    for _ in range(60):
        r.step(Twist2D(0.4, 0.8), 0.05, box_world)        # R = 0.5, center (3, 1.5)
    d = math.hypot(r.pose.x - 3.0, r.pose.y - 1.5)
    assert d == pytest.approx(0.5, abs=0.02)


def test_collision_stops_robot(box_world, cfg):
    r = DiffDriveRobot(cfg.robot, Pose2D(5.0, 3.9, 0.0))  # heading at east wall
    hit = False
    for _ in range(100):
        r.step(Twist2D(0.5, 0.0), 0.05, box_world)
        hit = hit or r.collided
    assert hit
    assert r.pose.x < 5.9 - cfg.robot.radius + 0.02       # never penetrates


def test_lidar_exact_ranges(box_world, cfg, rng):
    lidar = Lidar(cfg.lidar, rng)                          # noise_std = 0 via fixture
    scan = lidar.scan(box_world, Pose2D(1.0, 1.0, 0.0), 0.0)
    i_fwd = int(round((0.0 - scan.angle_min) / scan.angle_increment)) % scan.num_beams
    # forward beam (+x): wall inner face at x=5.9 → expected 4.9
    assert scan.ranges[i_fwd] == pytest.approx(4.9, abs=0.08)


def test_sim_odometry_drifts_but_tracks(box_world, cfg, rng):
    sim = Simulator(box_world, cfg, rng)
    for _ in range(100):
        res = sim.step(Twist2D(0.4, 0.3))
    gt, od = res.ground_truth, res.odom_pose
    err = math.hypot(gt.x - od.x, gt.y - od.y)
    assert 0.0 < err < 0.8
    assert res.sim_time == pytest.approx(5.0)
```

- [ ] **Step 2:** Run `tests/test_sim.py` — fails (modules missing).
- [ ] **Step 3:** Implement the four modules per the spec.
- [ ] **Step 4:** `.venv/bin/python -m pytest tests/test_sim.py -q` — all pass. Also sanity-check speed: a 100-step run with scans must take < 2 s wall.
- [ ] **Step 5:** Commit: `git commit -am "feat(sim): diff-drive world simulator with vectorized lidar and odom noise"`.

## Task 1.2 (Lane MAP): Mapping — `amr/mapping/{map_io,occupancy_grid_mapper}.py`

**Files:** Create both modules, `tests/test_map_io.py`, `tests/test_mapper.py`.

**Public API (frozen):**
```python
def save_map(grid: OccupancyGrid, path_stem: str) -> Tuple[str, str]: ...  # (pgm, yaml) paths
def load_map(yaml_path: str) -> OccupancyGrid: ...

class OccupancyGridMapper:
    log_odds: np.ndarray  # float32 (rows, cols), 0 = unobserved
    def __init__(self, cfg: MappingConfig, size_m: Tuple[float, float],
                 origin_xy: Tuple[float, float] = (0.0, 0.0)): ...
    def update(self, pose: Pose2D, scan: LaserScan) -> None: ...
    def to_occupancy_grid(self) -> OccupancyGrid: ...
```

**Spec:** PGM mapping per plan §2 item 6. Load thresholds: pixel ≤ 50 → `100`; pixel ≥ 200 → `0`;
else `-1`. Mapper update: for beams `[::cfg.beam_subsample]`: endpoint
`pose ⊕ (r·cosθ_b, r·sinθ_b)`; `bresenham` from robot cell to endpoint cell; all cells but the
last get `+= l_free`; the last gets `+= l_occ` **only if** the beam is valid (a real return);
for no-return beams trace free space to `range_max * 0.99` and skip the endpoint update.
Clamp to `±l_clamp`. `to_occupancy_grid`: `p = 1 - 1/(1 + exp(log_odds))`;
`p > occupied_thresh → 100`, `p < free_thresh → 0`, else `-1`.

- [ ] **Step 1: Failing tests.** `tests/test_map_io.py`:
```python
import numpy as np
import yaml

from amr.core.types import OccupancyGrid
from amr.mapping.map_io import load_map, save_map


def test_roundtrip(tmp_path):
    data = np.full((5, 7), -1, dtype=np.int8)
    data[0, 0] = 0
    data[4, 6] = 100
    data[2, 3] = 0
    g = OccupancyGrid(0.05, -1.0, 2.0, data)
    pgm, ypath = save_map(g, str(tmp_path / "m"))
    g2 = load_map(ypath)
    assert np.array_equal(g2.data, g.data)
    assert g2.resolution == g.resolution
    assert (g2.origin_x, g2.origin_y) == (g.origin_x, g.origin_y)
    meta = yaml.safe_load(open(ypath))
    assert meta["origin"] == [-1.0, 2.0, 0.0] and meta["negate"] == 0


def test_pgm_pixel_convention(tmp_path):
    data = np.array([[100, 0, -1]], dtype=np.int8)
    pgm, _ = save_map(OccupancyGrid(0.1, 0, 0, data), str(tmp_path / "m"))
    raw = open(pgm, "rb").read()
    body = raw.split(b"255", 1)[1].lstrip()[:3]          # after maxval header
    assert body[0] == 0 and body[1] == 254 and body[2] == 205
```
`tests/test_mapper.py`:
```python
import numpy as np

from amr.core.config import MappingConfig
from amr.core.types import LaserScan, Pose2D
from amr.mapping.occupancy_grid_mapper import OccupancyGridMapper


def _scan_hit_at(dist, n=8):
    return LaserScan(angle_min=-np.pi, angle_increment=2 * np.pi / n,
                     range_min=0.1, range_max=8.0,
                     ranges=np.full(n, dist))


def test_single_scan_free_occ_unknown():
    m = OccupancyGridMapper(MappingConfig(beam_subsample=1), (6.0, 6.0))
    for _ in range(4):                                    # strengthen evidence
        m.update(Pose2D(3.0, 3.0, 0.0), _scan_hit_at(2.0))
    g = m.to_occupancy_grid()
    assert g.data[g.world_to_grid(4.0, 3.0)] == 0         # on the +x beam, before hit
    assert g.data[g.world_to_grid(5.0, 3.0)] == 100       # the hit cell
    assert g.data[g.world_to_grid(5.5, 3.0)] == -1        # behind the hit
    assert g.data[g.world_to_grid(3.0, 5.0)] == 100       # +y beam hit
```
- [ ] **Step 2:** Run — fail. **Step 3:** Implement. **Step 4:** Pass.
- [ ] **Step 5:** Commit: `feat(mapping): log-odds occupancy mapper and ROS-compatible map I/O`.

## Task 1.3 (Lane PLAN): Costmap + A* — `amr/planning/{costmap,astar}.py`

**Files:** Create both modules, `tests/test_costmap.py`, `tests/test_astar.py`.

**Public API (frozen):**
```python
class Costmap:
    LETHAL = 1.0
    cost: np.ndarray          # float32 (rows, cols) in [0, 1]
    resolution: float
    origin_x: float
    origin_y: float
    def __init__(self, grid: OccupancyGrid, cfg: CostmapConfig, robot_radius: float): ...
    def is_lethal(self, row: int, col: int) -> bool: ...        # cost >= 0.99
    def cost_at_world(self, x: float, y: float) -> float: ...   # out of bounds → 1.0
    def world_to_grid / grid_to_world / in_bounds               # same semantics as OccupancyGrid

def plan_path(costmap: Costmap, start_xy, goal_xy, cfg: AstarConfig) -> Optional[np.ndarray]:
    """(M, 2) world waypoints start→goal, or None if unreachable."""

def has_line_of_sight(costmap: Costmap, p, q) -> bool:
    """True iff every sample at resolution/2 spacing along p→q is non-lethal."""
```

**Spec — Costmap:** obstacle mask = `data >= occupied_thresh`, plus `data < 0` if
`unknown_is_lethal`. `d = distance_field(mask, res, inflation_radius)` (from `amr.core.geometry`).
`cost = 1.0` where `d <= robot_radius`; `cost = exp(-cost_decay * (d - robot_radius))` where
`robot_radius < d < inflation_radius`; `0.0` beyond. Because inflation already accounts for the
robot radius, **all later collision checks treat the robot as a point**.

**Spec — A*:** 8-connected `heapq` search on cells. Step cost
`step_len * (1 + w_cost * cost[nbr])` with `step_len ∈ {1, √2}` (cell units); lethal cells
excluded. Heuristic: octile distance (admissible since the multiplier ≥ 1). If the start cell is
lethal, search outward (BFS ≤ 0.3 m) for the nearest non-lethal cell to start from; if the goal
cell is lethal → return None. Reconstruct to world points (cell centers), prepend exact start,
append exact goal. If `cfg.simplify`: greedy shortcutting — keep a point only if
`has_line_of_sight` fails to its successor's successor.

- [ ] **Step 1: Failing tests.** `tests/test_costmap.py`:
```python
from amr.core.config import CostmapConfig
from amr.planning.costmap import Costmap


def test_costmap_layers(box_world):
    cm = Costmap(box_world.grid, CostmapConfig(), robot_radius=0.18)
    r, c = cm.world_to_grid(3.0, 3.0)                     # inside the center box
    assert cm.is_lethal(r, c)
    assert cm.cost_at_world(3.0, 2.55) >= 0.99            # within radius of box edge
    near = cm.cost_at_world(3.0, 2.45)                    # inside inflation band
    far = cm.cost_at_world(3.0, 2.30)                     # further out, still in band
    assert 0.0 < far < near < 1.0
    assert cm.cost_at_world(1.0, 1.0) == 0.0
    assert cm.cost_at_world(99.0, 99.0) == 1.0
```
`tests/test_astar.py`:
```python
import math

import numpy as np

from amr.core.config import AstarConfig, CostmapConfig
from amr.planning.astar import has_line_of_sight, plan_path
from amr.planning.costmap import Costmap
from amr.sim.world import World


def test_path_around_box(box_world):
    cm = Costmap(box_world.grid, CostmapConfig(), 0.18)
    path = plan_path(cm, (1.0, 1.0), (5.0, 5.0), AstarConfig())
    assert path is not None
    assert np.allclose(path[0], [1.0, 1.0]) and np.allclose(path[-1], [5.0, 5.0])
    for x, y in path:
        assert cm.cost_at_world(x, y) < 0.99
    length = np.sum(np.hypot(*np.diff(path, axis=0).T))
    assert length >= math.hypot(4, 4) - 0.01


def test_no_path_when_sealed():
    d = dict(size=[4.0, 4.0], resolution=0.05, spawn={"x": 1, "y": 1, "theta": 0},
             obstacles=[{"type": "rect", "x": 1.9, "y": 0.0, "w": 0.2, "h": 4.0}])
    w = World.from_dict(d)
    cm = Costmap(w.grid, CostmapConfig(unknown_is_lethal=False), 0.18)
    assert plan_path(cm, (1.0, 2.0), (3.0, 2.0), AstarConfig()) is None


def test_line_of_sight(box_world):
    cm = Costmap(box_world.grid, CostmapConfig(), 0.18)
    assert has_line_of_sight(cm, (1.0, 1.0), (1.0, 5.0))
    assert not has_line_of_sight(cm, (1.0, 3.0), (5.0, 3.0))   # crosses the box
```
- [ ] **Step 2:** Fail. **Step 3:** Implement. **Step 4:** Pass (A* on the 120×120 box world must finish < 1 s).
- [ ] **Step 5:** Commit: `feat(planning): inflated costmap and A* global planner with path simplification`.

## Task 1.4 (Lane PLAN, after 1.3): DWA local planner — `amr/planning/dwa.py`

**Files:** Create `amr/planning/dwa.py`, `tests/test_dwa.py`.

**Public API (frozen):**
```python
@dataclass
class DwaResult:
    cmd: Twist2D
    trajectory: np.ndarray    # (K, 3) poses of the chosen rollout
    blocked: bool             # True if every sampled rollout collides

class DwaPlanner:
    def __init__(self, cfg: DwaConfig, robot: RobotConfig): ...
    def compute(self, pose: Pose2D, vel: Twist2D, path: np.ndarray,
                costmap: Costmap) -> DwaResult: ...

def carrot_point(path: np.ndarray, pose: Pose2D, lookahead: float) -> np.ndarray:
    """Point 'lookahead' metres of arc length beyond the projection of the pose
    onto the path polyline; clamps to the final point (the goal)."""
```

**Spec:** dynamic window `v ∈ [max(0, vel.v - a_lin·T), min(v_max, vel.v + a_lin·T)]`
(`T = sim_time` is used as the reachability horizon for sampling simplicity), `omega`
symmetric likewise; `v_samples × w_samples` linspace grid. Rollout: unicycle Euler at `sim_dt`
for `sim_time`. Reject any rollout containing a pose with `cost_at_world ≥ 0.99`. Near-goal
slowdown: if `dist(pose, goal) < lookahead`, cap sampled `v` at `max(0.1, 0.7·dist)`.
Scores per surviving rollout (each normalized to [0, 1] across rollouts before weighting):
`progress = -‖traj_end − carrot‖`, `heading = -|bearing from traj_end to carrot|`,
`clearance = min over rollout of (1 − cost)`, `velocity = v / v_max`. Total =
`w_progress·progress + w_heading·heading + w_clearance·clearance + w_velocity·velocity`.
All rollouts rejected → `DwaResult(Twist2D(0,0), empty, blocked=True)`.

- [ ] **Step 1: Failing tests** (`tests/test_dwa.py`):
```python
import numpy as np

from amr.core.config import CostmapConfig, DwaConfig, RobotConfig
from amr.core.types import Pose2D, Twist2D
from amr.planning.costmap import Costmap
from amr.planning.dwa import DwaPlanner, carrot_point


def _cm(box_world):
    return Costmap(box_world.grid, CostmapConfig(), 0.18)


def test_carrot():
    path = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    c = carrot_point(path, Pose2D(0.2, 0.1, 0.0), 0.8)
    assert c[0] >= 0.8 and abs(c[1]) < 1e-6
    assert np.allclose(carrot_point(path, Pose2D(1.9, 0, 0), 0.8), [2.0, 0.0])


def test_drives_forward_when_clear(box_world):
    dwa = DwaPlanner(DwaConfig(), RobotConfig())
    path = np.array([[1.0, 1.0], [2.0, 1.0], [3.0, 1.0]])
    res = dwa.compute(Pose2D(1.0, 1.0, 0.0), Twist2D(0.3, 0.0), path, _cm(box_world))
    assert not res.blocked and res.cmd.v > 0.15
    for x, y, _ in res.trajectory:
        assert _cm(box_world).cost_at_world(x, y) < 0.99


def test_avoids_wall_ahead(box_world):
    dwa = DwaPlanner(DwaConfig(), RobotConfig())
    pose = Pose2D(5.0, 3.0, 0.0)                          # 0.9 m from east wall
    path = np.array([[5.0, 3.0], [5.0, 4.5]])             # path bends north
    res = dwa.compute(pose, Twist2D(0.4, 0.0), path, _cm(box_world))
    assert not res.blocked
    assert res.cmd.omega > 0.1                            # turns left toward the path
    for x, y, _ in res.trajectory:
        assert _cm(box_world).cost_at_world(x, y) < 0.99
```
- [ ] **Step 2:** Fail. **Step 3:** Implement (vectorize rollouts if convenient; a Python loop
  over ≤ 120 rollouts × 15 steps is fine). **Step 4:** Pass; `compute` must run < 30 ms.
- [ ] **Step 5:** Commit: `feat(planning): DWA local planner with dynamic-window sampling`.

## Task 1.5 (Lane LOC): Probabilistic models — `amr/localization/{motion_model,sensor_model}.py`

**Files:** Create both modules, `tests/test_motion_sensor.py`.

**Public API (frozen):**
```python
def sample_motion(particles: np.ndarray, odom_delta: Pose2D,
                  alphas: Sequence[float], rng: np.random.Generator) -> np.ndarray:
    """Thrun odometry motion model applied to (N,3) particles; returns new (N,3)."""

class LikelihoodField:
    def __init__(self, grid: OccupancyGrid, cfg: LikelihoodConfig): ...
    def weigh(self, particles: np.ndarray, scan: LaserScan) -> np.ndarray:
        """(N,) unnormalized weights."""
```

**Spec — motion model:** decompose the robot-frame `odom_delta = (dx, dy, dth)`:
`trans = hypot(dx, dy)`; `rot1 = atan2(dy, dx)` if `trans > 1e-4` else `0`;
`rot2 = wrap(dth − rot1)`. Per-particle noisy samples (vectorized, size N):
`rot1' = rot1 + rng.normal(0, a1·|rot1| + a2·trans, N)`,
`trans' = trans + rng.normal(0, a3·trans + a4·(|rot1|+|rot2|), N)`,
`rot2' = rot2 + rng.normal(0, a1·|rot2| + a2·trans, N)`. Apply:
`x += trans'·cos(th + rot1')`, `y += trans'·sin(th + rot1')`, `th = wrap(th + rot1' + rot2')`.

**Spec — likelihood field:** precompute `distance_field(grid.data >= 65, res, max_dist)`.
`weigh`: take valid beams `[::beam_subsample]`; endpoints in the robot frame `(B, 2)`;
broadcast-transform by all particles → `(N, B, 2)` world points → integer cells;
out-of-bounds/unknown cells get `d = max_dist`. Per-beam likelihood
`q = z_hit · exp(−d²/(2σ²)) + z_rand / range_max`; weight `w_i = exp(Σ_b ln q_ib)`.

- [ ] **Step 1: Failing tests** (`tests/test_motion_sensor.py`):
```python
import numpy as np
import pytest

from amr.core.config import LikelihoodConfig
from amr.core.types import LaserScan, Pose2D
from amr.localization.motion_model import sample_motion
from amr.localization.sensor_model import LikelihoodField


def test_motion_model_mean_and_spread(rng):
    p = np.tile([2.0, 3.0, 0.0], (2000, 1))
    out = sample_motion(p, Pose2D(0.5, 0.0, 0.1), [0.05] * 4, rng)
    assert out[:, 0].mean() == pytest.approx(2.5, abs=0.02)
    assert out[:, 1].std() > 0.001                       # noise actually applied
    assert out[:, 2].mean() == pytest.approx(0.1, abs=0.02)


def test_likelihood_prefers_true_pose(box_world, cfg, rng):
    from amr.sim.lidar import Lidar
    truth = Pose2D(1.5, 1.5, 0.5)
    scan = Lidar(cfg.lidar, rng).scan(box_world, truth, 0.0)
    field = LikelihoodField(box_world.grid, LikelihoodConfig())
    particles = np.array([[1.5, 1.5, 0.5],
                          [1.5, 1.5, 1.1],
                          [3.9, 1.2, 0.5],
                          [1.0, 4.0, -2.0]])
    w = field.weigh(particles, scan)
    assert np.argmax(w) == 0
    assert w[0] > 5.0 * w[2]
```
- [ ] **Step 2:** Fail. **Step 3:** Implement. **Step 4:** Pass.
- [ ] **Step 5:** Commit: `feat(localization): odometry motion model and likelihood-field sensor model`.

### GATE B (end of Phase 1)
`.venv/bin/python -m pytest -q` → everything green. All four lanes merged/committed.

---

# Phase 2 — Estimation & decision (THREE PARALLEL LANES)

## Task 2.1 (needs SIM + MAP): Scan-matching SLAM — `amr/slam/scan_matching_slam.py`

**Files:** Create the module, `tests/test_slam.py`.

**Public API (frozen):**
```python
class ScanMatchingSlam:
    pose: Pose2D                      # current estimate in the map frame
    mapper: OccupancyGridMapper
    def __init__(self, cfg: SlamConfig, mapping_cfg: MappingConfig,
                 size_m: Tuple[float, float], initial_pose: Pose2D): ...
    def process(self, odom_delta: Pose2D, scan: Optional[LaserScan]) -> Pose2D: ...
    def get_map(self) -> OccupancyGrid: ...
```

**Spec (correlative scan matching against a blurred occupancy score map):**
1. `pose = pose_compose(pose, odom_delta)` every call. If `scan is None` → return.
2. First scan ever: `mapper.update(pose, scan)`, rebuild score map, remember
   `last_kf_pose = pose`, `accum = Pose2D()`, return.
3. Skip matching entirely if displacement since the last *processed* scan
   `< min_motion` (both trans and |rot|).
4. **Score map:** `p_occ` from `1 − 1/(1+exp(log_odds))`; `score_src = (p_occ > 0.6)` as float;
   blur with a separable Gaussian kernel (σ = `blur_sigma_cells`, half-width 3σ) implemented as
   a sum of `w_k · np.roll`-style shifted slices along each axis (no scipy); normalize peak to 1.
   Rebuild only after each map (keyframe) update.
5. **Two-stage search** over candidate offsets around the predicted pose, using
   `match_beams` evenly subsampled valid beams as robot-frame points `(B, 2)`:
   coarse grid `dx, dy ∈ ±coarse_window_xy step coarse_step_xy`,
   `dθ ∈ ±coarse_window_theta step coarse_step_theta`; then fine grid around the coarse best:
   `±coarse_step_xy step fine_step_xy`, `±coarse_step_theta step fine_step_theta`.
   Candidate score = mean over beams of `score_map[cell(endpoint)]` (out-of-map → 0).
   Vectorize: for each θ rotate points once `(B,2)`, then add the `(Kxy, 2)` translation grid
   via broadcasting → `(Kxy, B)` cell lookups per θ.
6. If best score ≥ `min_match_score`: `pose = best candidate`; else keep the odometry
   prediction (early map is too thin to trust matching).
7. **Keyframe:** if motion since `last_kf_pose` ≥ `keyframe_trans` or `keyframe_rot`:
   `mapper.update(pose, scan)`, rebuild the score map, update `last_kf_pose`.

- [ ] **Step 1: Failing test** (`tests/test_slam.py`):
```python
import math

import numpy as np
import pytest

from amr.core.types import Twist2D
from amr.slam.scan_matching_slam import ScanMatchingSlam
from amr.sim.simulator import Simulator
from amr.sim.world import World


@pytest.mark.slow
def test_slam_square_loop(cfg, rng):
    world = World.from_yaml("configs/worlds/office.yaml")
    cfg.lidar.noise_std = 0.01
    sim = Simulator(world, cfg, rng)
    slam = ScanMatchingSlam(cfg.slam, cfg.mapping, (12.0, 9.0), world.spawn)

    # drive a square-ish loop in the west room with open-loop commands
    legs = [(0.3, 0.0, 6.0), (0.0, math.pi / 8, 4.0)] * 4
    for v, w, dur in legs:
        for _ in range(int(dur / cfg.sim.dt)):
            res = sim.step(Twist2D(v, w))
            slam.process(res.odom_delta, res.scan)

    gt = sim.robot.pose
    err = math.hypot(slam.pose.x - gt.x, slam.pose.y - gt.y)
    raw_odom = res.odom_pose
    raw_err = math.hypot(raw_odom.x - gt.x, raw_odom.y - gt.y)
    assert err < 0.30                                     # bounded estimate error
    assert err <= raw_err + 0.05                          # no worse than raw odometry

    g = slam.get_map()
    occ_cells = np.argwhere(g.data >= 65)
    assert len(occ_cells) > 200                           # walls actually mapped
    # precision: mapped-occupied cells must hug true obstacles (within 2 cells)
    from amr.core.geometry import distance_field
    d_true = distance_field(world.grid.data >= 65, g.resolution, 1.0)
    hits = sum(1 for r, c in occ_cells if d_true[r, c] <= 2 * g.resolution + 1e-6)
    assert hits / len(occ_cells) > 0.85
```
- [ ] **Step 2:** Fail. **Step 3:** Implement per spec. **Step 4:** Pass; the whole test must
  finish < 60 s wall (tune by lowering `match_beams`, never by widening the error bound).
- [ ] **Step 5:** Commit: `feat(slam): correlative scan-matching SLAM with keyframed map updates`.

## Task 2.2 (needs LOC + SIM): Monte-Carlo localization — `amr/localization/mcl.py`

**Files:** Create the module, `tests/test_mcl.py`.

**Public API (frozen):**
```python
class MonteCarloLocalizer:
    particles: np.ndarray    # (N, 3)
    weights: np.ndarray      # (N,) normalized
    def __init__(self, grid: OccupancyGrid, cfg: LocalizationConfig,
                 rng: np.random.Generator,
                 initial_pose: Optional[Pose2D] = None): ...
    def predict(self, odom_delta: Pose2D) -> None: ...
    def correct(self, scan: LaserScan) -> None: ...
    def estimate(self) -> Pose2D: ...
    def set_pose(self, pose: Pose2D) -> None: ...        # re-init gaussian cloud
```

**Spec:** `initial_pose=None` → global init: sample uniformly over free cells (`data == 0`),
θ uniform. Otherwise gaussian around the pose with `init_std`. `predict` = `sample_motion`.
`correct`: `weights *= LikelihoodField.weigh(...)`; renormalize (guard: if the sum underflows
to 0, reset to uniform); resample with the **low-variance resampler** when
`1/Σw² < resample_neff_frac · N`. Resampler (include verbatim):
```python
def _low_variance_resample(particles, weights, rng):
    n = len(weights)
    positions = (rng.random() + np.arange(n)) / n
    idx = np.searchsorted(np.cumsum(weights), positions)
    return particles[np.minimum(idx, n - 1)].copy()
```
`estimate`: weighted means for x, y; `theta = atan2(Σw·sinθ, Σw·cosθ)`.

- [ ] **Step 1: Failing test** (`tests/test_mcl.py`):
```python
import math

import pytest

from amr.core.types import Pose2D, Twist2D
from amr.localization.mcl import MonteCarloLocalizer
from amr.sim.simulator import Simulator


@pytest.mark.slow
def test_mcl_tracks_through_motion(box_world, cfg, rng):
    cfg.lidar.noise_std = 0.01
    sim = Simulator(box_world, cfg, rng)
    mcl = MonteCarloLocalizer(box_world.grid, cfg.localization, rng,
                              initial_pose=box_world.spawn)
    script = [(0.3, 0.0, 4.0), (0.0, 0.6, 2.5), (0.3, 0.0, 4.0), (0.0, 0.6, 2.5),
              (0.3, 0.0, 4.0)]
    for v, w, dur in script:
        for _ in range(int(dur / cfg.sim.dt)):
            res = sim.step(Twist2D(v, w))
            mcl.predict(res.odom_delta)
            if res.scan is not None:
                mcl.correct(res.scan)
    est, gt = mcl.estimate(), sim.robot.pose
    assert math.hypot(est.x - gt.x, est.y - gt.y) < 0.25
    assert abs(math.remainder(est.theta - gt.theta, 2 * math.pi)) < 0.25


def test_set_pose_resets_cloud(box_world, cfg, rng):
    mcl = MonteCarloLocalizer(box_world.grid, cfg.localization, rng,
                              initial_pose=Pose2D(1, 1, 0))
    mcl.set_pose(Pose2D(4.0, 4.0, 1.0))
    assert abs(mcl.particles[:, 0].mean() - 4.0) < 0.1
    e = mcl.estimate()
    assert (round(e.x, 0), round(e.y, 0)) == (4.0, 4.0)
```
- [ ] **Step 2:** Fail. **Step 3:** Implement. **Step 4:** Pass (< 30 s wall).
- [ ] **Step 5:** Commit: `feat(localization): MCL particle filter with adaptive low-variance resampling`.

## Task 2.3 (needs PLAN): Navigation state machine — `amr/navigation/navigator.py`

**Files:** Create the module, `tests/test_navigator.py`.

**Public API (frozen):**
```python
class NavState(Enum):
    IDLE = "IDLE"; PLANNING = "PLANNING"; FOLLOWING = "FOLLOWING"
    RECOVERY = "RECOVERY"; SUCCEEDED = "SUCCEEDED"; FAILED = "FAILED"

class Navigator:
    state: NavState
    goal: Optional[Pose2D]
    path: Optional[np.ndarray]
    def __init__(self, cfg: NavConfig, astar_cfg: AstarConfig, dwa: DwaPlanner): ...
    def set_goal(self, goal: Pose2D) -> None: ...
    def cancel(self) -> None: ...
    def update(self, pose: Pose2D, vel: Twist2D, costmap: Costmap,
               now: float) -> Twist2D: ...
```

**Spec — transition table (implement exactly):**

| State | Condition in `update` | Action / next state |
|---|---|---|
| IDLE / SUCCEEDED / FAILED | — | return `Twist2D(0,0)` |
| PLANNING | `plan_path` succeeds | store path + `plan_time=now` → FOLLOWING |
| PLANNING | plan fails | `recovery_count += 1`; if > `max_recoveries` → FAILED else → RECOVERY |
| FOLLOWING | `dist(pose, goal) < goal_tol_xy` | → SUCCEEDED, zero cmd |
| FOLLOWING | `now − plan_time > replan_period` **or** any path point within `path_block_check_dist` ahead is lethal | replan inline (same tick); on failure follow the PLANNING-failure row |
| FOLLOWING | `dwa.compute(...).blocked` | → RECOVERY (`recovery_count += 1`; FAILED if exceeded) |
| FOLLOWING | otherwise | return DWA cmd |
| RECOVERY | behavior incomplete | behavior 0: rotate in place at `recovery_rotate_speed` until accumulated rotation ≥ 2π; behavior 1: back up `recovery_backup_dist` at `−recovery_backup_speed`; alternate per entry |
| RECOVERY | behavior complete | → PLANNING |

RECOVERY progress is integrated from successive `now` values (the caller's clock), not from
pose feedback: rotate until `Σ recovery_rotate_speed·Δnow ≥ 2π`; back up until
`Σ recovery_backup_speed·Δnow ≥ recovery_backup_dist`.

`set_goal` resets `recovery_count = 0`, state → PLANNING. `cancel` → IDLE, clears goal/path.
Log every transition at INFO via `get_logger("nav")`.

- [ ] **Step 1: Failing tests** (`tests/test_navigator.py`):
```python
import math

import pytest

from amr.core.config import AstarConfig, CostmapConfig, DwaConfig, RobotConfig, NavConfig
from amr.core.types import Pose2D, Twist2D
from amr.navigation.navigator import Navigator, NavState
from amr.planning.costmap import Costmap
from amr.planning.dwa import DwaPlanner
from amr.sim.robot import DiffDriveRobot


def _nav():
    return Navigator(NavConfig(), AstarConfig(), DwaPlanner(DwaConfig(), RobotConfig()))


@pytest.mark.slow
def test_reaches_goal_in_box_world(box_world, cfg):
    cm = Costmap(box_world.grid, CostmapConfig(), cfg.robot.radius)
    nav = _nav()
    robot = DiffDriveRobot(cfg.robot, box_world.spawn)
    nav.set_goal(Pose2D(5.0, 5.0, 0.0))
    t = 0.0
    while t < 60.0 and nav.state not in (NavState.SUCCEEDED, NavState.FAILED):
        cmd = nav.update(robot.pose, robot.vel, cm, t)
        robot.step(cmd, 0.05, box_world)
        t += 0.05
    assert nav.state == NavState.SUCCEEDED
    assert math.hypot(robot.pose.x - 5.0, robot.pose.y - 5.0) < NavConfig().goal_tol_xy + 0.05
    assert not robot.collided


def test_unreachable_goal_fails_after_recoveries(box_world, cfg):
    cm = Costmap(box_world.grid, CostmapConfig(), cfg.robot.radius)
    nav = _nav()
    nav.set_goal(Pose2D(3.0, 3.0, 0.0))                   # inside the center box
    states = set()
    t = 0.0
    while t < 120.0 and nav.state != NavState.FAILED:
        nav.update(Pose2D(1.0, 1.0, 0.0), Twist2D(), cm, t)
        states.add(nav.state)
        t += 0.05
    assert nav.state == NavState.FAILED
    assert NavState.RECOVERY in states


def test_idle_and_cancel(box_world, cfg):
    cm = Costmap(box_world.grid, CostmapConfig(), cfg.robot.radius)
    nav = _nav()
    assert nav.update(Pose2D(1, 1, 0), Twist2D(), cm, 0.0) == Twist2D(0.0, 0.0)
    nav.set_goal(Pose2D(5, 5, 0))
    nav.cancel()
    assert nav.state == NavState.IDLE and nav.goal is None
```
- [ ] **Step 2:** Fail. **Step 3:** Implement. **Step 4:** Pass.
- [ ] **Step 5:** Commit: `feat(navigation): goal-driven FSM with replanning and recovery behaviors`.

### GATE C (end of Phase 2)
`.venv/bin/python -m pytest -q` → all green, including the `slow` estimation tests.

---

# Phase 3 — Integration (SERIAL)

## Task 3.1: Runtime conductor + CLI — `amr/runtime/app.py`, `amr/cli.py`, mission file

**Files:** Create `amr/runtime/app.py`, `configs/missions/office_mapping.yaml`,
rewrite `amr/cli.py`, `tests/test_cli.py`.

**Public API (frozen — the GUI consumes exactly this):**
```python
class Mode(Enum):
    SLAM = "SLAM"; NAV = "NAV"

@dataclass
class Snapshot:
    mode: Mode
    nav_state: str               # NavState value or "MAPPING"
    pose: Pose2D                 # current best estimate (SLAM pose or MCL estimate)
    gt_pose: Pose2D              # simulator ground truth (display/metrics only)
    scan_points: Optional[np.ndarray]   # (K, 2) world-frame scan endpoints
    particles: Optional[np.ndarray]     # (N, 3) in NAV mode, else None
    path: Optional[np.ndarray]          # (M, 2)
    goal: Optional[Pose2D]
    grid: Optional[OccupancyGrid]       # live SLAM map or loaded static map
    map_version: int             # bump on every map change; GUI re-renders on change
    sim_time: float
    collided: bool
    estop: bool
    status: str                  # short human-readable line for the status bar

# Commands (plain dataclasses; AmrApp.handle_command dispatches on type):
@dataclass
class SetGoal:        x: float; y: float
@dataclass
class SetInitialPose: x: float; y: float; theta: float
@dataclass
class SaveMap:        path_stem: str
@dataclass
class SetMode:        mode: Mode          # NAV requires a loaded/just-saved map
@dataclass
class EStop:          pass
@dataclass
class Resume:         pass
@dataclass
class SetManual:      enabled: bool
@dataclass
class SetTeleop:      v: float; omega: float
@dataclass
class Shutdown:       pass

class WaypointDriver:
    def __init__(self, waypoints: List[Tuple[float, float]]): ...
    done: bool
    def update(self, pose: Pose2D, scan: Optional[LaserScan]) -> Twist2D: ...

class AmrApp:
    def __init__(self, cfg: AmrConfig, mode: Mode,
                 map_path: Optional[str] = None,
                 mission_file: Optional[str] = None): ...
    def step(self) -> Snapshot: ...
    def handle_command(self, cmd) -> None: ...
    def run_headless(self, max_sim_time: float,
                     stop_when=None) -> Snapshot: ...   # stop_when: Snapshot -> bool
```

**Spec:**
- Seeding: one `np.random.default_rng(cfg.seed)` created in `__init__`, shared by sim/MCL.
- **SLAM-mode step order:** `cmd = driver.update(slam.pose, last_scan)` (or teleop/zero) →
  `res = sim.step(cmd)` → `slam.process(res.odom_delta, res.scan)`. When the mission driver
  finishes, `status = "mission_complete"` and cmd = zero.
- **NAV-mode step order:** `res = sim.step(prev_cmd)` → `mcl.predict(res.odom_delta)` →
  if scan: `mcl.correct` → `pose = mcl.estimate()` → `cmd = navigator.update(pose, robot.vel,
  costmap, sim_time)`. The costmap is built **once** from the loaded map (rebuild only on
  `SetInitialPose`-independent map reload). `nav_state = navigator.state.value`.
  NAV-mode construction: the robot spawns at `world.spawn` and MCL is initialized with
  `initial_pose = world.spawn` (gaussian cloud, `init_std`); `SetInitialPose` re-seeds the
  cloud via `mcl.set_pose`.
- E-stop forces zero cmd and freezes the navigator (no state advance) until `Resume`.
- Manual mode: teleop twist replaces the autonomous cmd (works in both modes).
- `SetMode(NAV)` mid-session: requires a map (loaded via constructor `map_path` or
  `SaveMap`-then-reload of the live SLAM map); builds MCL initialized at the current SLAM
  pose estimate, builds costmap/navigator. This is the GUI's "finish mapping, start navigating"
  flow.
- `run_headless`: loop `step()` until `stop_when(snapshot)` or `sim_time ≥ max_sim_time`;
  returns the last snapshot. No sleeping — run flat out.
- WaypointDriver: P-controller — `bearing = wrap(atan2(dy,dx) − pose.theta)`;
  if `|bearing| > 0.5`: `Twist2D(0.05, 1.2·sign(bearing))` else
  `Twist2D(min(0.35, 0.8·dist), 1.5·bearing)`; waypoint reached at `dist < 0.35` → advance.
  Safety: if any valid scan range within ±25° of forward is `< 0.30` → force `v = 0`
  (keep ω). All waypoints in the mission below were checked ≥ 0.45 m clear of obstacles
  along straight segments — if the driver still clips a wall in practice, nudge the offending
  waypoint by ≤ 0.3 m and re-verify; do not weaken the safety stop.

`configs/missions/office_mapping.yaml` (complete):
```yaml
waypoints:
  - [2.5, 1.5]
  - [2.5, 6.3]
  - [5.0, 6.25]    # through the west door (gap x≈4.07, y 5.5–7.0)
  - [6.0, 3.0]
  - [6.5, 1.5]
  - [9.5, 1.5]     # through the south gap past wall B
  - [10.8, 2.0]
  - [10.8, 7.8]
  - [10.8, 2.0]
  - [9.5, 1.5]
  - [6.0, 2.5]
  - [5.0, 5.6]
  - [4.5, 6.25]
  - [2.5, 6.0]
  - [2.0, 2.0]
```

**CLI (rewrite `amr/cli.py`):** global flags `--config PATH` (default `configs/default.yaml`
if it exists, else dataclass defaults), `--set KEY=VAL` (repeatable), `--log-level LEVEL`.
Subcommands:
- `amr slam --mission PATH --out STEM [--max-time S=240]` → SLAM mission, save map, print
  final pose error vs ground truth, exit 0 on `mission_complete`.
- `amr nav --map YAML --goal X,Y [--goal X,Y ...] [--max-time S=180]` → navigate each goal in
  sequence; exit 0 iff every goal SUCCEEDED.
- `amr demo [--out-dir DIR=maps]` → full pipeline (create DIR with
  `os.makedirs(..., exist_ok=True)` first): SLAM mission → save `DIR/office` →
  NAV on the saved map to `(10.5, 1.5)` then `(2.0, 7.0)` → print `DEMO PASS`/`DEMO FAIL`,
  exit code 0/1.
- `amr gui [--map YAML]` → Task 4.1's GUI (SLAM mode when no map given, NAV mode with map).
`setup_logging` is called once in `main` after config resolution.

- [ ] **Step 1: Failing tests** (`tests/test_cli.py`):
```python
import os

import pytest

from amr import cli


def test_help_and_version():
    with pytest.raises(SystemExit) as e:
        cli.main(["--help"])
    assert e.value.code == 0


def test_slam_smoke_writes_map(tmp_path):
    rc = cli.main(["--set", "logging.console=false",
                   "slam", "--mission", "configs/missions/office_mapping.yaml",
                   "--out", str(tmp_path / "m"), "--max-time", "5"])
    # 5 sim-seconds: mission not complete -> nonzero, but map files must exist
    assert (tmp_path / "m.pgm").exists() and (tmp_path / "m.yaml").exists()
    assert rc != 0


def test_nav_smoke_on_truth_map(tmp_path, box_world):
    from amr.mapping.map_io import save_map
    save_map(box_world.grid, str(tmp_path / "box"))
    rc = cli.main(["--set", "logging.console=false",
                   "--set", "sim.world_file=tests/box_world.yaml",
                   "nav", "--map", str(tmp_path / "box.yaml"),
                   "--goal", "5.0,5.0", "--max-time", "60"])
    assert rc == 0
```
Also create `tests/box_world.yaml` mirroring `BOX_WORLD` from `conftest.py` (same numbers, YAML
syntax — used by the CLI test which needs a world *file*).

- [ ] **Step 2:** Fail. **Step 3:** Implement `runtime/app.py` then `cli.py`.
- [ ] **Step 4:** `pytest tests/test_cli.py -q` green.
- [ ] **Step 5:** Commit: `feat(runtime,cli): AmrApp conductor with snapshot/command interface and CLI`.

## Task 3.2: End-to-end autonomy test — `tests/test_integration_e2e.py`

**Files:** Create `tests/test_integration_e2e.py`.

- [ ] **Step 1: Write the test** (complete):
```python
import math

import pytest

from amr.core.config import load_config
from amr.runtime.app import AmrApp, Mode, SaveMap, SetGoal


@pytest.mark.slow
def test_full_autonomy_pipeline(tmp_path):
    cfg = load_config("configs/default.yaml",
                      overrides=["logging.console=false",
                                 "localization.num_particles=400"])

    # ---- Phase 1: autonomous SLAM mapping mission -------------------------
    app = AmrApp(cfg, Mode.SLAM,
                 mission_file="configs/missions/office_mapping.yaml")
    snap = app.run_headless(240.0, stop_when=lambda s: s.status == "mission_complete")
    assert snap.status == "mission_complete", "mapping mission did not finish in time"
    slam_err = math.hypot(snap.pose.x - snap.gt_pose.x, snap.pose.y - snap.gt_pose.y)
    assert slam_err < 0.30
    stem = str(tmp_path / "office")
    app.handle_command(SaveMap(stem))

    # ---- Phase 2: localize + navigate on the SLAM-built map ---------------
    app2 = AmrApp(cfg, Mode.NAV, map_path=stem + ".yaml")
    deadline = 180.0                                      # sim-seconds for goal 1
    for gx, gy in [(10.5, 1.5), (2.0, 7.0)]:
        app2.handle_command(SetGoal(gx, gy))
        snap = app2.run_headless(
            deadline, stop_when=lambda s: s.nav_state in ("SUCCEEDED", "FAILED"))
        assert snap.nav_state == "SUCCEEDED", "failed to reach (%s, %s)" % (gx, gy)
        true_err = math.hypot(snap.gt_pose.x - gx, snap.gt_pose.y - gy)
        assert true_err < cfg.nav.goal_tol_xy + 0.15      # honest, ground-truth check
        assert not snap.collided
        deadline = snap.sim_time + 180.0                  # ceiling is absolute sim time
```
(`run_headless`'s `max_sim_time` is an **absolute** sim-time ceiling; the `deadline`
bookkeeping above gives each goal 180 sim-seconds of budget.)

- [ ] **Step 2:** Run: `.venv/bin/python -m pytest tests/test_integration_e2e.py -q -x`.
- [ ] **Step 3:** Debug the full stack until it passes. Likely first failures and where to look:
  mission timeout → WaypointDriver safety stop oscillation (check waypoint clearances);
  SLAM error too big → score-map blur σ or search window too small; nav FAILED → costmap
  `unknown_is_lethal` interaction with map specks (consider a 1-cell despeckle: occupied cells
  with no occupied neighbor → unknown, applied in `Costmap.__init__`).
  **This step loops until green — do not relax assertions to make it pass.**
- [ ] **Step 4:** Run `.venv/bin/amr demo` → must print `DEMO PASS` and exit 0.
- [ ] **Step 5:** Commit: `test(e2e): full SLAM→save→localize→navigate autonomy pipeline`.

### GATE D (end of Phase 3)
`.venv/bin/python -m pytest -q && .venv/bin/amr demo` → all tests green **and** `DEMO PASS`.

---

# Phase 4 — HRI GUI, deployment, documentation (THREE PARALLEL LANES)

## Task 4.1 (Lane GUI): tkinter HRI application — `amr/gui/{app,map_canvas,panels}.py`

**Files:** Create the three modules, `tests/test_gui_smoke.py`.

**Threading architecture (strict):** the worker thread owns `AmrApp` and loops
`app.step()` paced to `dt / speed_factor` wall seconds; it publishes every snapshot into a
single-slot `Mailbox` (a `threading.Lock`-guarded "latest value" holder — implement in
`amr/gui/app.py`) and drains a `queue.Queue` of commands into `app.handle_command`.
**Only the Tk main thread touches widgets**; it polls the mailbox every `cfg.gui.refresh_ms`
via `root.after`. Window close → put `Shutdown`, `join` the worker, then `root.destroy()`.

**Map rendering** (`map_canvas.py`, include this helper verbatim):
```python
def grid_to_photoimage(grid, zoom):
    """OccupancyGrid -> tk.PhotoImage via an in-memory PGM (P5), no PIL needed."""
    g = np.full(grid.data.shape, 128, dtype=np.uint8)   # unknown: gray
    g[grid.data == 0] = 230                             # free: light
    g[grid.data >= 65] = 30                             # occupied: dark
    g = np.flipud(g)                                    # image row 0 = top
    h, w = g.shape
    header = ("P5 %d %d 255 " % (w, h)).encode("ascii")
    img = tk.PhotoImage(data=header + g.tobytes(), format="PPM")
    return img.zoom(zoom, zoom)
```
(Tk's PPM decoder accepts P5; if a Tk build rejects it, fall back to P6 by repeating the gray
channel 3×.) Re-render only when `snapshot.map_version` changes; keep a reference to the
PhotoImage on the widget to defeat garbage collection.

**Canvas transform:** `cx = (x − origin_x)/res · zoom`, `cy = (rows − (y − origin_y)/res) · zoom`;
implement `canvas_to_world` as the exact inverse for mouse events.

**Overlays per refresh** (delete-by-tag then redraw): robot footprint circle + heading line
(estimate, blue) and ground truth (gray, toggleable); lidar points (≤ 120, red dots); particles
(≤ 300 short heading ticks, green); global path (orange polyline); goal marker (cross).

**Panels (`panels.py`):**
- Mode: `[SLAM Mapping] [Navigate]` buttons → `SetMode` (Navigate prompts to save the current
  SLAM map first via `SaveMap` when entering NAV from SLAM).
- Actions: `[Save Map] [E-STOP] [Resume] [Manual ✓]`, speed scale 0.5–4× (sets worker pacing).
- Status bar: `nav_state | pose x,y,θ | sim time | E-STOP flag | collision flag` from the
  snapshot's `status` plus formatted fields.
- Log panel: `tk.Text` fed by a `logging.Handler` subclass that pushes formatted records into a
  `queue.Queue` (attached via `amr.core.log.attach_handler`); the Tk loop drains ≤ 50 records
  per refresh, keeps the last 500 lines, colors WARN/ERROR via text tags.
- Mouse: left-click on canvas in NAV mode → `SetGoal`; click-drag in "Set Pose" toggle mode →
  `SetInitialPose` (drag direction = heading). Keys: WASD/arrows → `SetTeleop` while Manual.

- [ ] **Step 1: Failing smoke test** (`tests/test_gui_smoke.py`):
```python
import os

import numpy as np
import pytest

tk = pytest.importorskip("tkinter")

from amr.core.config import load_config
from amr.core.types import OccupancyGrid, Pose2D
from amr.gui.map_canvas import grid_to_photoimage
from amr.runtime.app import Mode, Snapshot

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"),
                                reason="no display available")


def _fake_snapshot(grid):
    return Snapshot(mode=Mode.SLAM, nav_state="MAPPING", pose=Pose2D(1, 1, 0),
                    gt_pose=Pose2D(1, 1, 0),
                    scan_points=np.array([[2.0, 2.0], [1.5, 0.5]]),
                    particles=None, path=None, goal=None, grid=grid,
                    map_version=1, sim_time=0.0, collided=False, estop=False,
                    status="smoke")


def test_render_pipeline_headless_window():
    from amr.gui.app import AmrGuiApp
    cfg = load_config()
    grid = OccupancyGrid(0.05, 0, 0,
                         np.zeros((60, 60), dtype=np.int8))
    grid.data[0, :] = 100
    gui = AmrGuiApp(cfg, start_worker=False)              # constructor must allow this
    gui.root.withdraw()
    img = grid_to_photoimage(grid, 2)
    assert img.width() == 120 and img.height() == 120     # 60 cells × zoom 2
    gui.render_snapshot(_fake_snapshot(grid))             # public for testability
    items = gui.canvas.find_all()
    assert len(items) >= 3                                # map image + robot + scan
    ps = gui.canvas.postscript()                          # proves real drawing happened
    assert len(ps) > 500
    gui.root.destroy()
```
- [ ] **Step 2:** Fail. **Step 3:** Implement the three modules. `AmrGuiApp.__init__(cfg,
  start_worker=True, mode=Mode.SLAM, map_path=None)`; `render_snapshot(snapshot)` is the
  single entry the refresh loop calls — keep it pure-ish for the test.
- [ ] **Step 4:** `pytest tests/test_gui_smoke.py -q` green (WSLg display is available on this
  machine).
- [ ] **Step 5 (manual visual check, WSLg):** run `timeout 25 .venv/bin/amr gui` — the window
  must show the office map building up while the robot drives the mission; press E-STOP and
  confirm the robot freezes; close cleanly (no traceback, no hung process).
- [ ] **Step 6:** Commit: `feat(gui): tkinter HRI console with live map, goal setting, teleop, e-stop, log panel`.

## Task 4.2 (Lane DEPLOY): Deployment artifacts — `deploy/`

**Files:** Create `deploy/Dockerfile`, `deploy/docker-compose.yaml`, `deploy/amr.service`,
`deploy/install.sh`.

- [ ] **Step 1: Author files** (complete contents):

`deploy/Dockerfile` (ubuntu base so tkinter exists for the GUI variant; build context = repo root):
```dockerfile
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-tk python3-venv \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/amr
COPY pyproject.toml setup.py README.md ./
COPY amr/ amr/
COPY configs/ configs/
RUN python3 -m pip install --no-cache-dir .
ENTRYPOINT ["amr"]
CMD ["demo"]
```

`deploy/docker-compose.yaml`:
```yaml
services:
  amr-demo:
    build:
      context: ..
      dockerfile: deploy/Dockerfile
    command: demo
  amr-gui:
    build:
      context: ..
      dockerfile: deploy/Dockerfile
    command: gui
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    network_mode: host
```

`deploy/amr.service` (demonstrates service deployment; on real hardware ExecStart would point
at the hardware bringup instead of the sim demo):
```ini
[Unit]
Description=AMR Stack autonomous navigation service
After=network.target

[Service]
Type=simple
WorkingDirectory=/opt/amr
ExecStart=/opt/amr/.venv/bin/amr demo
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

`deploy/install.sh`:
```bash
#!/usr/bin/env bash
# Install the AMR stack to a target machine directory and register the
# systemd unit. Usage: sudo deploy/install.sh [DEST=/opt/amr]
set -euo pipefail
DEST="${1:-/opt/amr}"
SRC="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$DEST"
rsync -a --exclude .venv --exclude .git --exclude logs "$SRC/" "$DEST/"
python3 -m venv --system-site-packages "$DEST/.venv"
"$DEST/.venv/bin/pip" install --upgrade pip
"$DEST/.venv/bin/pip" install "$DEST"
install -m 644 "$DEST/deploy/amr.service" /etc/systemd/system/amr.service
systemctl daemon-reload
echo "Installed. Enable with: systemctl enable --now amr.service"
```

- [ ] **Step 2: Validate** (docker and systemd-as-PID-1 are unavailable on this dev box —
  static validation only, and say so in the final report):
  `bash -n deploy/install.sh && bash -n scripts/setup.sh` (exit 0);
  `python3 -c "import configparser; c=configparser.ConfigParser(); c.read('deploy/amr.service'); assert c['Service']['Restart']=='on-failure'"`;
  `python3 -c "import yaml; yaml.safe_load(open('deploy/docker-compose.yaml'))"`.
- [ ] **Step 3:** Commit: `feat(deploy): dockerfile, compose, systemd unit, installer`.

## Task 4.3 (Lane DOCS): Documentation + README

**Files:** Create `docs/architecture.md`, `docs/configuration.md`, `docs/user_guide.md`,
rewrite `README.md`.

- [ ] **Step 1:** `README.md` — project pitch, capability table (reuse plan §4), quickstart:
  ```
  make setup      # venv + editable install
  make test       # full test suite
  make demo       # autonomous SLAM → map → navigate pipeline (prints DEMO PASS)
  make gui        # HRI console (needs a display)
  ```
  plus a CLI reference for `amr slam/nav/demo/gui` and the `--config/--set` mechanism.
- [ ] **Step 2:** `docs/architecture.md` — module diagram (ASCII), data flow for both modes
  (the two step-order lists from Task 3.1 spec), coordinate conventions (copy plan §2),
  algorithm summaries with the chosen parameters and *why* (one paragraph each: log-odds
  mapping, correlative scan matching, MCL likelihood field, costmap inflation, A*, DWA, FSM).
  State explicitly: loop closure is out of scope for v0.1; office-scale drift is handled by
  scan matching.
- [ ] **Step 3:** `docs/configuration.md` — a table per config section: key, type, default,
  meaning, which module reads it. Source of truth = the dataclasses in Task 0.3 (keep values
  in sync; the `default.yaml` equality test guards the YAML side).
- [ ] **Step 4:** `docs/user_guide.md` — GUI walkthrough (every widget and mouse/key binding
  from Task 4.1), headless CLI recipes, map file format, deployment guide (docker compose,
  install.sh + systemd), troubleshooting (no DISPLAY, slow SLAM → lower `match_beams`,
  flaky localization → raise `num_particles`).
- [ ] **Step 5:** Commit: `docs: architecture, configuration reference, user guide, README`.
- [ ] **Step 6 (vault):** Save `architecture.md` and `user_guide.md` (and the final report from
  Gate E) into the Obsidian vault at `/home/cona/kangj/general_vault` using the
  **`obsidian-vault-save` skill** (user-level skill; follow its conventions for frontmatter,
  placement, and the vault git commit). This step is mandatory — the user wants all project
  documentation reachable from Obsidian.

### GATE E (FINAL — loop until green)
Run, in order, fixing and re-running until all four pass in one session:
1. `bash scripts/setup.sh` (from a clean `.venv` removal — proves "setups" works end to end)
2. `.venv/bin/python -m pytest -q` — **0 failures, 0 errors** (GUI test included under WSLg)
3. `.venv/bin/amr demo` — prints `DEMO PASS`, exit 0
4. `timeout 25 .venv/bin/amr gui` manual visual check (window renders, no traceback on close)

Then: `git tag v0.1.0`, and write a short completion report (what was validated, what was
deferred: docker build, systemd runtime) — save it to the vault per Task 4.3 Step 6.

---

# Appendix A — Orchestration guide for the Opus dynamic workflow

**Dependency graph (tasks may start the moment their inputs are merged):**
```
0.1 → 0.2 → 0.3 → 0.4                          (serial; one agent is fine)
GATE A
├── 1.1 SIM lane          (needs core only)
├── 1.2 MAP lane          (needs core only)
├── 1.3 → 1.4 PLAN lane   (needs core only; 1.4 after 1.3)
└── 1.5 LOC lane          (needs core only)
GATE B
├── 2.1 SLAM   (needs 1.1 + 1.2)
├── 2.2 MCL    (needs 1.1 + 1.5; map fixture from core types only)
└── 2.3 NAV    (needs 1.3 + 1.4; sim robot from 1.1 for its integration test)
GATE C
3.1 → 3.2 (serial — the integration owner should be your strongest agent)
GATE D
├── 4.1 GUI    (needs 3.1's Snapshot/Command contract)
├── 4.2 DEPLOY (independent)
└── 4.3 DOCS   (independent; vault save at the end)
GATE E (single agent, loop until green)
```

**Rules for subagents:**
1. **Interface freeze:** the signatures/dataclasses marked *frozen* in this plan are the
   contract. If a change is unavoidable, the lane stops, the orchestrator updates the plan
   section and *all* dependent lanes are notified before anyone continues.
2. One checkout, disjoint files per lane: each lane edits only its listed files plus its own
   tests, and commits only those (`git add <explicit paths>`). conftest.py belongs to lane SIM
   (Task 1.1); other lanes only read it.
3. Every task ends with its own pytest selection green and a commit. Gates run the full suite.
4. TDD is non-negotiable: write the test first exactly as printed, watch it fail, implement,
   watch it pass. Never weaken an assertion to get past a gate; tune algorithm parameters
   instead.
5. Subagent model hints: algorithmic tasks (2.1, 2.2, 3.1, 3.2, 4.1) deserve the strongest
   model available; 0.1, 4.2, 4.3 are mechanical.
6. Performance budgets are requirements: full suite < 4 min wall on this machine; `amr demo`
   < 3 min wall. If exceeded, vectorize (lidar, likelihood field, scan matching are the
   hotspots) — do not silently shrink test coverage.
7. Python 3.8 discipline (no `match`, no `dict |`, no builtin-generic annotations without the
   `__future__` import) and numpy-1.17 discipline (only ops listed in the plan).
   `grep -rnE "match |removeprefix|removesuffix" amr/` before each gate is a cheap guard,
   and the venv's 3.8 interpreter will catch the rest at import time.
8. The final fix-loop (Gate E) is explicitly authorized to touch any file, but every fix must
   keep all prior tests green — run the full suite after each change.

# Appendix B — Risk register & known gotchas

| Risk | Mitigation already baked into the plan |
|---|---|
| Probabilistic test flakiness | Single seeded `default_rng(cfg.seed)` everywhere; generous-but-meaningful bounds; `slow` marker isolates them |
| Python-loop performance (lidar / MCL / scan matching) | Vectorized specs given (argmax-first-hit raycast, broadcast likelihood field, per-θ broadcast matching); budgets in Gate rules |
| PGM y-flip / coordinate bugs | Conventions frozen in §2; round-trip and pixel-level tests in Task 1.2; cell-center semantics tested in Task 0.2 |
| `from __future__ import annotations` breaks `get_type_hints` on configs | config.py deliberately avoids the future import and resolves via `get_type_hints` (Task 0.3 code is complete — use as written) |
| tkinter cross-thread crashes | Strict mailbox/queue architecture in Task 4.1; only the Tk thread touches widgets |
| SLAM diverges early (empty map) | `min_match_score` gate keeps odometry until the map has structure |
| Costmap kills paths via map speckle | Optional 1-cell despeckle noted in Task 3.2 Step 3 |
| venv/pip quirks on Ubuntu 20.04 | `--system-site-packages` + pip upgrade + `setup.py` shim (Tasks 0.1, 4.2 installer) |
| No docker/systemd on dev box | Static validation commands provided (Task 4.2); deferred items must be listed in the completion report |

# Appendix C — Documentation-to-vault requirement

Per the user's standing instruction, **all standalone documentation produced by this project
must be saved into the Obsidian vault** (`/home/cona/kangj/general_vault`) via the
`obsidian-vault-save` skill: this plan (already saved), `docs/architecture.md`,
`docs/user_guide.md`, and the Gate E completion report. Use the skill's conventions (frontmatter,
folder placement, vault git commit) rather than copying files by hand.
