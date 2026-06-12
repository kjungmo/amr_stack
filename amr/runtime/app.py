"""Runtime conductor: wires every subsystem behind a Snapshot/Command interface.

The :class:`AmrApp` is the single integration layer used identically by the
headless CLI and (later) the tkinter GUI. It runs in one of two modes:

* ``Mode.SLAM`` -- a waypoint mission drives the simulated robot while
  :class:`ScanMatchingSlam` builds a live map.
* ``Mode.NAV`` -- on a loaded (or just-saved) static map, MCL localizes the
  robot while the :class:`Navigator` drives it to goals.

Every ``step()`` returns an immutable-ish :class:`Snapshot` describing the full
observable state. Commands are plain dataclasses dispatched by
``handle_command``. The whole stack is deterministic: a single
``np.random.default_rng(cfg.seed)`` created in ``__init__`` is shared by the
simulator and MCL.

Coordinate / data conventions follow plan §2.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

from amr.core.config import AmrConfig
from amr.core.geometry import transform_points, wrap_angle
from amr.core.log import get_logger
from amr.core.types import LaserScan, OccupancyGrid, Pose2D, Twist2D
from amr.mapping.map_io import load_map, save_map
from amr.localization.mcl import MonteCarloLocalizer
from amr.navigation.navigator import Navigator
from amr.planning.costmap import Costmap
from amr.planning.dwa import DwaPlanner
from amr.sim.simulator import Simulator
from amr.sim.world import World
from amr.slam.scan_matching_slam import ScanMatchingSlam


class Mode(Enum):
    SLAM = "SLAM"
    NAV = "NAV"


# Extra clearance (m) added to the robot radius when inflating the *planning*
# costmap in NAV mode. The Navigator's integration test drives on a perfect
# ground-truth pose, but the real NAV runtime tracks with MCL, whose ~1 cm
# estimate lag is enough to graze tight obstacle corners when A* hugs the
# inflation band. A small planning margin keeps global paths off razor-thin
# corners without narrowing real doorways (office gaps are >= 0.7 m clear).
_NAV_PLANNING_MARGIN = 0.06


# ---------------------------------------------------------------------------
# Snapshot
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Commands (plain dataclasses; AmrApp.handle_command dispatches on type)
# ---------------------------------------------------------------------------

@dataclass
class SetGoal:
    x: float
    y: float


@dataclass
class SetInitialPose:
    x: float
    y: float
    theta: float


@dataclass
class SaveMap:
    path_stem: str


@dataclass
class SetMode:
    mode: Mode          # NAV requires a loaded/just-saved map


@dataclass
class EStop:
    pass


@dataclass
class Resume:
    pass


@dataclass
class SetManual:
    enabled: bool


@dataclass
class SetTeleop:
    v: float
    omega: float


@dataclass
class Shutdown:
    pass


# ---------------------------------------------------------------------------
# WaypointDriver
# ---------------------------------------------------------------------------

class WaypointDriver:
    """P-controller that drives a list of (x, y) waypoints with a safety stop.

    Control law (plan §Task 3.1):
      ``bearing = wrap(atan2(dy, dx) - pose.theta)``;
      if ``|bearing| > 0.5``: ``Twist2D(0.05, 1.2 * sign(bearing))``
      else                  : ``Twist2D(min(0.35, 0.8 * dist), 1.5 * bearing)``.
    A waypoint is reached at ``dist < 0.35`` -> advance to the next one.
    Safety: if any valid scan range within +-25 deg of forward is < 0.30 m,
    force ``v = 0`` (keep omega).
    """

    def __init__(self, waypoints: List[Tuple[float, float]]):
        self._waypoints = [(float(x), float(y)) for x, y in waypoints]
        self._idx = 0
        self.done = len(self._waypoints) == 0

    def update(self, pose: Pose2D, scan: Optional[LaserScan]) -> Twist2D:
        if self.done or self._idx >= len(self._waypoints):
            self.done = True
            return Twist2D(0.0, 0.0)

        wx, wy = self._waypoints[self._idx]
        dx = wx - pose.x
        dy = wy - pose.y
        dist = math.hypot(dx, dy)

        # Reached the current waypoint -> advance (possibly to completion).
        if dist < 0.35:
            self._idx += 1
            if self._idx >= len(self._waypoints):
                self.done = True
                return Twist2D(0.0, 0.0)
            wx, wy = self._waypoints[self._idx]
            dx = wx - pose.x
            dy = wy - pose.y
            dist = math.hypot(dx, dy)

        bearing = wrap_angle(math.atan2(dy, dx) - pose.theta)
        if abs(bearing) > 0.5:
            cmd = Twist2D(0.05, 1.2 * (1.0 if bearing >= 0.0 else -1.0))
        else:
            cmd = Twist2D(min(0.35, 0.8 * dist), 1.5 * bearing)

        # Safety stop: any valid forward (+-25 deg) range below 0.30 m.
        if scan is not None and self._forward_blocked(scan):
            cmd = Twist2D(0.0, cmd.omega)

        return cmd

    @staticmethod
    def _forward_blocked(scan: LaserScan) -> bool:
        angles = scan.angles()
        ranges = np.asarray(scan.ranges, dtype=float)
        valid = scan.valid_mask()
        cone = np.abs(wrap_angle_vec(angles)) <= math.radians(25.0)
        sel = valid & cone
        if not np.any(sel):
            return False
        return bool(np.any(ranges[sel] < 0.30))


def wrap_angle_vec(a: np.ndarray) -> np.ndarray:
    """Vector wrap to (-pi, pi] for the forward-cone test."""
    a = np.asarray(a, dtype=float)
    out = np.mod(a + math.pi, 2.0 * math.pi)
    out = np.where(out <= 0.0, out + 2.0 * math.pi, out)
    return out - math.pi


# ---------------------------------------------------------------------------
# AmrApp
# ---------------------------------------------------------------------------

class AmrApp:
    """Runtime conductor. See module docstring and plan §Task 3.1."""

    def __init__(self, cfg: AmrConfig, mode: Mode,
                 map_path: Optional[str] = None,
                 mission_file: Optional[str] = None):
        self.cfg = cfg
        self.mode = mode
        self._log = get_logger("runtime")

        # Single shared generator (deterministic): sim + MCL draw from it.
        self.rng = np.random.default_rng(cfg.seed)

        # Simulated world (ground truth) is always present.
        self.world = World.from_yaml(cfg.sim.world_file)
        self.sim = Simulator(self.world, cfg, self.rng)

        # Live observable state.
        self.last_scan: Optional[LaserScan] = None
        self.sim_time = 0.0
        self.collided = False
        self.estop = False
        self.manual = False
        self.teleop = Twist2D(0.0, 0.0)
        self.map_version = 0
        self.status = ""
        self._shutdown = False

        # Subsystems set per-mode.
        self.slam: Optional[ScanMatchingSlam] = None
        self.driver: Optional[WaypointDriver] = None
        self.mcl: Optional[MonteCarloLocalizer] = None
        self.navigator: Optional[Navigator] = None
        self.costmap: Optional[Costmap] = None
        self.static_grid: Optional[OccupancyGrid] = None
        self.prev_cmd = Twist2D(0.0, 0.0)
        self.goal: Optional[Pose2D] = None

        self._mission_file = mission_file
        self._map_path = map_path

        if mode == Mode.SLAM:
            self._init_slam(mission_file)
        elif mode == Mode.NAV:
            if map_path is None:
                raise ValueError("NAV mode requires map_path")
            grid = load_map(map_path)
            self._init_nav(grid, initial_pose=self.world.spawn)
        else:  # pragma: no cover - enum is closed
            raise ValueError("unknown mode %r" % (mode,))

    # ---------------------------------------------------- mode construction
    def _init_slam(self, mission_file: Optional[str]) -> None:
        self.slam = ScanMatchingSlam(self.cfg.slam, self.cfg.mapping,
                                     self.world.size, self.world.spawn)
        waypoints: List[Tuple[float, float]] = []
        if mission_file is not None:
            waypoints = _load_mission(mission_file)
        self.driver = WaypointDriver(waypoints)
        self.map_version += 1
        self.status = "mapping"

    def _init_nav(self, grid: OccupancyGrid, initial_pose: Pose2D) -> None:
        self.static_grid = grid
        self.mcl = MonteCarloLocalizer(grid, self.cfg.localization, self.rng,
                                       initial_pose=initial_pose)
        dwa = DwaPlanner(self.cfg.planning.dwa, self.cfg.robot)
        self.navigator = Navigator(self.cfg.nav, self.cfg.planning.astar, dwa)
        # Costmap is built once from the loaded static map. Inflate with a
        # small planning margin beyond the robot radius (see _NAV_PLANNING_MARGIN).
        self.costmap = Costmap(grid, self.cfg.planning.costmap,
                               self.cfg.robot.radius + _NAV_PLANNING_MARGIN)
        self.prev_cmd = Twist2D(0.0, 0.0)
        self.map_version += 1
        self.status = "navigating"

    # -------------------------------------------------------------- step()
    def step(self) -> Snapshot:
        if self.mode == Mode.SLAM:
            return self._step_slam()
        return self._step_nav()

    def _step_slam(self) -> Snapshot:
        assert self.slam is not None and self.driver is not None

        # 1. Decide the command: teleop (manual) / mission driver / zero.
        if self.estop:
            cmd = Twist2D(0.0, 0.0)
        elif self.manual:
            cmd = Twist2D(self.teleop.v, self.teleop.omega)
        else:
            cmd = self.driver.update(self.slam.pose, self.last_scan)

        # 2. Advance the simulator.
        res = self.sim.step(cmd)
        self.sim_time = res.sim_time
        self.collided = res.collided

        # 3. Fuse odometry + scan into SLAM.
        self.slam.process(res.odom_delta, res.scan)
        if res.scan is not None:
            self.last_scan = res.scan
            self.map_version += 1

        # Status line.
        if not self.manual and not self.estop and self.driver.done:
            self.status = "mission_complete"
        elif self.estop:
            self.status = "estop"
        elif self.manual:
            self.status = "manual"
        else:
            self.status = "mapping"

        grid = self.slam.get_map()
        return Snapshot(
            mode=self.mode,
            nav_state="MAPPING",
            pose=_clone_pose(self.slam.pose),
            gt_pose=_clone_pose(res.ground_truth),
            scan_points=self._scan_points(self.slam.pose, self.last_scan),
            particles=None,
            path=None,
            goal=_clone_pose(self.goal) if self.goal is not None else None,
            grid=grid,
            map_version=self.map_version,
            sim_time=self.sim_time,
            collided=self.collided,
            estop=self.estop,
            status=self.status,
        )

    def _step_nav(self) -> Snapshot:
        assert (self.mcl is not None and self.navigator is not None
                and self.costmap is not None)

        # 1. Advance the simulator with the previous command.
        if self.estop:
            self.prev_cmd = Twist2D(0.0, 0.0)
        res = self.sim.step(self.prev_cmd)
        self.sim_time = res.sim_time
        self.collided = res.collided

        # 2. MCL predict from the noisy odometry increment.
        self.mcl.predict(res.odom_delta)

        # 3. MCL correct when a scan is available.
        if res.scan is not None:
            self.mcl.correct(res.scan)
            self.last_scan = res.scan

        # 4. Pose estimate.
        pose = self.mcl.estimate()

        # 5. Compute the next command.
        if self.estop:
            # Frozen: no navigator state advance.
            cmd = Twist2D(0.0, 0.0)
        elif self.manual:
            cmd = Twist2D(self.teleop.v, self.teleop.omega)
        else:
            cmd = self.navigator.update(pose, self.sim.robot.vel,
                                        self.costmap, self.sim_time)
        self.prev_cmd = cmd

        nav_state = self.navigator.state.value
        if self.estop:
            self.status = "estop"
        elif self.manual:
            self.status = "manual"
        else:
            self.status = nav_state.lower()

        return Snapshot(
            mode=self.mode,
            nav_state=nav_state,
            pose=_clone_pose(pose),
            gt_pose=_clone_pose(res.ground_truth),
            scan_points=self._scan_points(pose, self.last_scan),
            particles=self.mcl.particles.copy(),
            path=None if self.navigator.path is None else np.asarray(
                self.navigator.path, dtype=float).copy(),
            goal=_clone_pose(self.goal) if self.goal is not None else None,
            grid=self.static_grid,
            map_version=self.map_version,
            sim_time=self.sim_time,
            collided=self.collided,
            estop=self.estop,
            status=self.status,
        )

    # ------------------------------------------------------ handle_command
    def handle_command(self, cmd) -> None:
        if isinstance(cmd, SetGoal):
            self._cmd_set_goal(cmd)
        elif isinstance(cmd, SetInitialPose):
            self._cmd_set_initial_pose(cmd)
        elif isinstance(cmd, SaveMap):
            self._cmd_save_map(cmd)
        elif isinstance(cmd, SetMode):
            self._cmd_set_mode(cmd)
        elif isinstance(cmd, EStop):
            self.estop = True
            self.prev_cmd = Twist2D(0.0, 0.0)
            self._log.info("runtime: E-STOP engaged")
        elif isinstance(cmd, Resume):
            self.estop = False
            self._log.info("runtime: resumed")
        elif isinstance(cmd, SetManual):
            self.manual = bool(cmd.enabled)
            if not self.manual:
                self.teleop = Twist2D(0.0, 0.0)
        elif isinstance(cmd, SetTeleop):
            self.teleop = Twist2D(float(cmd.v), float(cmd.omega))
        elif isinstance(cmd, Shutdown):
            self._shutdown = True
        else:
            raise ValueError("unknown command %r" % (cmd,))

    def _cmd_set_goal(self, cmd: SetGoal) -> None:
        self.goal = Pose2D(float(cmd.x), float(cmd.y), 0.0)
        if self.navigator is not None:
            self.navigator.set_goal(self.goal)

    def _cmd_set_initial_pose(self, cmd: SetInitialPose) -> None:
        pose = Pose2D(float(cmd.x), float(cmd.y), float(cmd.theta))
        if self.mcl is not None:
            self.mcl.set_pose(pose)

    def _cmd_save_map(self, cmd: SaveMap) -> None:
        if self.slam is not None:
            grid = self.slam.get_map()
        elif self.static_grid is not None:
            grid = self.static_grid
        else:  # pragma: no cover - one of the two always exists
            raise RuntimeError("no map to save")
        save_map(grid, cmd.path_stem)
        self._log.info("runtime: saved map to %s.{pgm,yaml}", cmd.path_stem)

    def _cmd_set_mode(self, cmd: SetMode) -> None:
        target = cmd.mode
        if target == self.mode:
            return
        if target == Mode.NAV:
            # Requires a map: prefer the live SLAM map; else a loaded map.
            if self.slam is not None:
                grid = self.slam.get_map()
                initial = _clone_pose(self.slam.pose)
            elif self.static_grid is not None:
                grid = self.static_grid
                initial = self.world.spawn
            else:  # pragma: no cover
                raise RuntimeError("SetMode(NAV) requires a map")
            self.mode = Mode.NAV
            self.slam = None
            self.driver = None
            self._init_nav(grid, initial_pose=initial)
        elif target == Mode.SLAM:
            self.mode = Mode.SLAM
            self.mcl = None
            self.navigator = None
            self.costmap = None
            self.static_grid = None
            self._init_slam(self._mission_file)

    # ----------------------------------------------------- run_headless
    def run_headless(self, max_sim_time: float, stop_when=None) -> Snapshot:
        """Loop step() flat out until stop_when or the sim-time ceiling.

        ``max_sim_time`` is an absolute sim-time ceiling. No sleeping.
        """
        snap = self.step()
        while snap.sim_time < max_sim_time:
            if stop_when is not None and stop_when(snap):
                break
            if self._shutdown:
                break
            snap = self.step()
        return snap

    # --------------------------------------------------------- helpers
    def _scan_points(self, pose: Pose2D,
                     scan: Optional[LaserScan]) -> Optional[np.ndarray]:
        if scan is None:
            return None
        angles = scan.angles()
        ranges = np.asarray(scan.ranges, dtype=float)
        valid = scan.valid_mask()
        if not np.any(valid):
            return np.zeros((0, 2), dtype=float)
        a = angles[valid]
        r = ranges[valid]
        pts = np.column_stack((r * np.cos(a), r * np.sin(a)))
        return transform_points(pose, pts)


# ---------------------------------------------------------------------------
# Module helpers
# ---------------------------------------------------------------------------

def _load_mission(path: str) -> List[Tuple[float, float]]:
    import yaml
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    wps = d.get("waypoints", []) or []
    return [(float(p[0]), float(p[1])) for p in wps]


def _clone_pose(p: Pose2D) -> Pose2D:
    return Pose2D(p.x, p.y, p.theta)
