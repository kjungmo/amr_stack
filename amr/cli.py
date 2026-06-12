"""AMR stack command-line interface.

Global flags resolve a configuration first; ``setup_logging`` is then called
exactly once before any subcommand runs. Subcommands:

* ``slam`` -- run a SLAM mission, save the resulting map, report pose error.
* ``nav``  -- localize + navigate a sequence of goals on a static map.
* ``demo`` -- the full pipeline (SLAM -> save -> navigate), prints DEMO PASS/FAIL.
* ``gui``  -- launch the tkinter HRI console (imports ``amr.gui`` lazily).

The ``slam``/``nav``/``demo`` subcommands import cleanly without ``amr.gui``
present; only ``gui`` touches it, and guards against its absence.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from typing import List, Optional, Tuple

from amr.core.config import apply_overrides, load_config
from amr.core.log import get_logger, setup_logging
from amr.runtime.app import (AmrApp, Mode, SaveMap, SetGoal)

_DEFAULT_CONFIG = "configs/default.yaml"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="amr", description="AMR stack CLI")
    parser.add_argument("--version", action="version", version="amr-stack 0.1.0")
    parser.add_argument("--config", default=None,
                        help="config YAML (default: configs/default.yaml if present)")
    parser.add_argument("--set", dest="overrides", action="append", default=[],
                        metavar="KEY=VAL", help="dotted config override (repeatable)")
    parser.add_argument("--log-level", default=None,
                        help="override logging.level (e.g. DEBUG, INFO)")

    sub = parser.add_subparsers(dest="command")

    p_slam = sub.add_parser("slam", help="run a SLAM mission and save the map")
    p_slam.add_argument("--mission", required=True, help="mission YAML with waypoints")
    p_slam.add_argument("--out", required=True, help="output map stem")
    p_slam.add_argument("--max-time", type=float, default=240.0,
                        help="sim-time ceiling (s)")

    p_nav = sub.add_parser("nav", help="localize and navigate goals on a map")
    p_nav.add_argument("--map", required=True, help="map YAML to navigate on")
    p_nav.add_argument("--goal", action="append", default=[], metavar="X,Y",
                       help="goal coordinate (repeatable)")
    p_nav.add_argument("--max-time", type=float, default=180.0,
                       help="sim-time budget per goal (s)")

    p_demo = sub.add_parser("demo", help="full SLAM -> save -> navigate pipeline")
    p_demo.add_argument("--out-dir", default="maps", help="output directory")

    p_gui = sub.add_parser("gui", help="launch the tkinter HRI console")
    p_gui.add_argument("--map", default=None, help="map YAML (NAV mode if given)")

    return parser


def _resolve_config(args):
    path = args.config
    if path is None and os.path.exists(_DEFAULT_CONFIG):
        path = _DEFAULT_CONFIG
    overrides = list(args.overrides or [])
    if args.log_level is not None:
        overrides.append("logging.level=%s" % args.log_level)
    return load_config(path, overrides=overrides)


def _parse_goal(text: str) -> Tuple[float, float]:
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError("goal must be X,Y; got %r" % text)
    return float(parts[0]), float(parts[1])


def main(argv: Optional[List[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    cfg = _resolve_config(args)
    setup_logging(cfg.logging)
    log = get_logger("cli")

    if args.command is None:
        parser.print_help()
        return 0
    if args.command == "slam":
        return _cmd_slam(cfg, args, log)
    if args.command == "nav":
        return _cmd_nav(cfg, args, log)
    if args.command == "demo":
        return _cmd_demo(cfg, args, log)
    if args.command == "gui":
        return _cmd_gui(cfg, args, log)
    parser.print_help()
    return 0


def _cmd_slam(cfg, args, log) -> int:
    app = AmrApp(cfg, Mode.SLAM, mission_file=args.mission)
    snap = app.run_headless(args.max_time,
                            stop_when=lambda s: s.status == "mission_complete")
    app.handle_command(SaveMap(args.out))
    err = math.hypot(snap.pose.x - snap.gt_pose.x, snap.pose.y - snap.gt_pose.y)
    log.info("slam: final pose error vs ground truth = %.3f m", err)
    print("slam: pose error %.3f m, status=%s" % (err, snap.status))
    if snap.status == "mission_complete":
        return 0
    log.warning("slam: mission did not complete within %.1f s", args.max_time)
    return 1


def _cmd_nav(cfg, args, log) -> int:
    goals = [_parse_goal(g) for g in (args.goal or [])]
    if not goals:
        log.error("nav: at least one --goal X,Y is required")
        return 1
    app = AmrApp(cfg, Mode.NAV, map_path=args.map)
    deadline = 0.0
    for gx, gy in goals:
        app.handle_command(SetGoal(gx, gy))
        deadline += args.max_time
        snap = app.run_headless(
            deadline, stop_when=lambda s: s.nav_state in ("SUCCEEDED", "FAILED"))
        if snap.nav_state != "SUCCEEDED":
            log.warning("nav: goal (%.2f, %.2f) -> %s", gx, gy, snap.nav_state)
            print("nav: goal (%.2f, %.2f) FAILED (%s)" % (gx, gy, snap.nav_state))
            return 1
        true_err = math.hypot(snap.gt_pose.x - gx, snap.gt_pose.y - gy)
        log.info("nav: reached goal (%.2f, %.2f), true error %.3f m",
                 gx, gy, true_err)
        print("nav: goal (%.2f, %.2f) SUCCEEDED, true error %.3f m"
              % (gx, gy, true_err))
    return 0


def _cmd_demo(cfg, args, log) -> int:
    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, "office")

    # Phase 1: autonomous SLAM mapping mission.
    app = AmrApp(cfg, Mode.SLAM, mission_file="configs/missions/office_mapping.yaml")
    snap = app.run_headless(240.0,
                            stop_when=lambda s: s.status == "mission_complete")
    if snap.status != "mission_complete":
        log.error("demo: mapping mission did not finish")
        print("DEMO FAIL")
        return 1
    app.handle_command(SaveMap(stem))

    # Phase 2: localize + navigate on the SLAM-built map.
    app2 = AmrApp(cfg, Mode.NAV, map_path=stem + ".yaml")
    deadline = 0.0
    for gx, gy in [(10.5, 1.5), (2.0, 7.0)]:
        app2.handle_command(SetGoal(gx, gy))
        deadline += 180.0
        snap = app2.run_headless(
            deadline, stop_when=lambda s: s.nav_state in ("SUCCEEDED", "FAILED"))
        if snap.nav_state != "SUCCEEDED":
            log.error("demo: failed to reach (%.1f, %.1f)", gx, gy)
            print("DEMO FAIL")
            return 1
        if snap.collided:
            log.error("demo: collision en route to (%.1f, %.1f)", gx, gy)
            print("DEMO FAIL")
            return 1
    print("DEMO PASS")
    return 0


def _cmd_gui(cfg, args, log) -> int:
    try:
        from amr.gui.app import AmrGuiApp
    except ImportError as exc:
        log.error("gui: the GUI module is unavailable (%s)", exc)
        print("amr gui: GUI is not available in this build (%s)" % exc,
              file=sys.stderr)
        return 1
    mode = Mode.NAV if args.map else Mode.SLAM
    gui = AmrGuiApp(cfg, start_worker=True, mode=mode, map_path=args.map)
    gui.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
