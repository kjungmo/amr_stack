"""tkinter HRI console: live map, goal setting, teleop, E-STOP, log panel.

Threading architecture (strict)
-------------------------------
* A **worker thread** owns the :class:`~amr.runtime.app.AmrApp`. It loops
  ``app.step()`` paced to ``dt / speed_factor`` wall-seconds, publishes every
  resulting :class:`~amr.runtime.app.Snapshot` into a single-slot
  :class:`Mailbox`, and drains a ``queue.Queue`` of commands into
  ``app.handle_command`` between steps.
* The **Tk main thread** is the *only* thread that touches widgets. It polls
  the mailbox every ``cfg.gui.refresh_ms`` via ``root.after``, renders the
  latest snapshot, and drains the log queue. User actions enqueue commands.
* Window close enqueues :class:`~amr.runtime.app.Shutdown`, joins the worker,
  then destroys the root.

Coordinate / data conventions follow plan §2.
"""
from __future__ import annotations

import os
import queue
import threading
import time
from typing import Optional

import tkinter as tk
from tkinter import ttk

from amr.core.config import AmrConfig
from amr.core.log import attach_handler, get_logger
from amr.gui import map_canvas
from amr.gui.panels import ControlPanel, LogPanel, QueueLogHandler
from amr.runtime.app import (AmrApp, EStop, Mode, Resume, SaveMap, SetGoal,
                             SetInitialPose, SetManual, SetMode, SetTeleop,
                             Shutdown, Snapshot)


# ---------------------------------------------------------------------------
# Mailbox: single-slot, lock-guarded latest-value holder
# ---------------------------------------------------------------------------

class Mailbox:
    """A thread-safe single-slot "latest value" holder.

    The worker thread ``put``s the most recent snapshot; the Tk thread ``get``s
    it. There is no queueing: a new value overwrites the old, so the GUI always
    renders the freshest state and never falls behind.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value = None  # type: Optional[Snapshot]

    def put(self, value: Snapshot) -> None:
        with self._lock:
            self._value = value

    def get(self) -> Optional[Snapshot]:
        with self._lock:
            return self._value


# ---------------------------------------------------------------------------
# Worker thread: owns AmrApp, paces step(), publishes snapshots
# ---------------------------------------------------------------------------

class _Worker(threading.Thread):
    """Owns the AmrApp and the simulation loop.

    Pacing: each iteration sleeps so the loop advances roughly ``dt`` of sim
    time per ``dt / speed_factor`` wall-seconds. ``speed_factor <= 0`` runs flat
    out (no sleep). The factor is read under a lock so the Tk thread can change
    it live via the speed scale.
    """

    def __init__(self, app: AmrApp, mailbox: Mailbox,
                 cmd_queue: "queue.Queue", dt: float, speed_factor: float):
        super().__init__(name="amr-worker", daemon=True)
        self._app = app
        self._mailbox = mailbox
        self._cmd_queue = cmd_queue
        self._dt = float(dt)
        self._speed_lock = threading.Lock()
        self._speed_factor = float(speed_factor)
        # NB: must not be named ``_stop`` — that shadows Thread._stop, an
        # internal method join() calls during teardown.
        self._stop_evt = threading.Event()
        self._log = get_logger("gui.worker")

    def set_speed_factor(self, factor: float) -> None:
        with self._speed_lock:
            self._speed_factor = float(factor)

    def _wall_period(self) -> float:
        with self._speed_lock:
            sf = self._speed_factor
        if sf <= 0.0:
            return 0.0
        return self._dt / sf

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        try:
            # Publish an initial snapshot so the GUI has something to draw.
            self._mailbox.put(self._app.step())
        except Exception:  # pragma: no cover - first step should not fail
            self._log.exception("gui worker: initial step failed")
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            if self._drain_commands():
                break  # Shutdown handled -> exit loop
            try:
                snap = self._app.step()
            except Exception:  # pragma: no cover - keep the GUI alive
                self._log.exception("gui worker: step raised; stopping loop")
                break
            self._mailbox.put(snap)
            period = self._wall_period()
            if period > 0.0:
                elapsed = time.monotonic() - t0
                remaining = period - elapsed
                if remaining > 0.0:
                    # Sleep in slices so stop() is honored promptly.
                    self._stop_evt.wait(remaining)

    def _drain_commands(self) -> bool:
        """Apply queued commands. Returns True if a Shutdown was seen."""
        saw_shutdown = False
        while True:
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            try:
                self._app.handle_command(cmd)
            except Exception:  # pragma: no cover - bad command shouldn't crash
                self._log.exception("gui worker: handle_command failed: %r", cmd)
            if isinstance(cmd, Shutdown):
                saw_shutdown = True
        return saw_shutdown


# ---------------------------------------------------------------------------
# AmrGuiApp
# ---------------------------------------------------------------------------

class AmrGuiApp:
    """The HRI console window and its worker.

    With ``start_worker=False`` the AmrApp / worker thread are *not* created;
    the window is built and :meth:`render_snapshot` can be driven directly
    (used by the smoke test). With ``start_worker=True`` (the live ``amr gui``
    path) the worker is launched and the Tk refresh loop polls the mailbox.
    """

    # Teleop velocities applied while a key is held in Manual mode.
    _TELEOP_V = 0.35
    _TELEOP_W = 1.2

    def __init__(self, cfg: AmrConfig, start_worker: bool = True,
                 mode: Mode = Mode.SLAM, map_path: Optional[str] = None):
        self.cfg = cfg
        self._mode = mode
        self._map_path = map_path
        self._log = get_logger("gui")

        self._zoom = int(cfg.gui.px_per_cell)
        self._refresh_ms = int(cfg.gui.refresh_ms)

        # Cross-thread plumbing.
        self.mailbox = Mailbox()
        self._cmd_queue = queue.Queue()  # type: queue.Queue
        self._log_queue = queue.Queue()  # type: queue.Queue
        self._worker = None  # type: Optional[_Worker]

        # Rendering state.
        self._view = None  # type: Optional[map_canvas.WorldView]
        self._map_version = -1
        self._map_image = None  # PhotoImage; kept on self to defeat GC.
        self._current_snapshot = None  # type: Optional[Snapshot]

        # Pose-set drag state (canvas pixels).
        self._setpose_drag = None  # type: Optional[tuple]

        # --- Build the window --------------------------------------------
        self.root = tk.Tk()
        self.root.title("AMR HRI Console")
        self._build_ui()

        # Route 'amr' log records into the on-screen panel.
        self._log_handler = QueueLogHandler(self._log_queue)
        attach_handler(self._log_handler)

        # --- Worker / AmrApp ---------------------------------------------
        self.app = None  # type: Optional[AmrApp]
        if start_worker:
            self._start_worker()
            self.root.after(self._refresh_ms, self._on_refresh)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------- UI build
    def _build_ui(self) -> None:
        root = self.root
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        body = ttk.Frame(root)
        body.grid(row=0, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        # Canvas (left, expands) inside a scrollable frame.
        canvas_frame = ttk.Frame(body)
        canvas_frame.grid(row=0, column=0, sticky="nsew")
        canvas_frame.columnconfigure(0, weight=1)
        canvas_frame.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(canvas_frame, background="#202830",
                                highlightthickness=0, width=600, height=600)
        hbar = ttk.Scrollbar(canvas_frame, orient="horizontal",
                             command=self.canvas.xview)
        vbar = ttk.Scrollbar(canvas_frame, orient="vertical",
                             command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")

        # Right column: control panel + log panel.
        side = ttk.Frame(body)
        side.grid(row=0, column=1, sticky="ns")
        self.controls = ControlPanel(
            side,
            on_set_mode_slam=self._action_mode_slam,
            on_set_mode_nav=self._action_mode_nav,
            on_save_map=self._action_save_map,
            on_estop=self._action_estop,
            on_resume=self._action_resume,
            on_manual_toggle=self._action_manual,
            on_setpose_toggle=self._action_setpose,
            on_speed=self._action_speed,
            initial_speed=float(self.cfg.gui.speed_factor) or 1.0,
        )
        self.controls.frame.pack(side="top", fill="x")
        self.log_panel = LogPanel(side, self._log_queue)
        self.log_panel.frame.pack(side="top", fill="both", expand=True,
                                  padx=6, pady=(0, 6))

        # Mouse + keyboard bindings.
        self.canvas.bind("<Button-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)
        for key in ("w", "a", "s", "d", "W", "A", "S", "D",
                    "Up", "Down", "Left", "Right"):
            root.bind("<KeyPress-%s>" % key, self._on_key_press)
            root.bind("<KeyRelease-%s>" % key, self._on_key_release)

    # --------------------------------------------------------- worker mgmt
    def _start_worker(self) -> None:
        self.app = AmrApp(
            self.cfg, self._mode, map_path=self._map_path,
            mission_file="configs/missions/office_mapping.yaml"
            if self._mode == Mode.SLAM and self._mission_available() else None)
        sf = float(self.cfg.gui.speed_factor) or 1.0
        self._worker = _Worker(self.app, self.mailbox, self._cmd_queue,
                               dt=self.cfg.sim.dt, speed_factor=sf)
        self._worker.start()

    @staticmethod
    def _mission_available() -> bool:
        return os.path.exists("configs/missions/office_mapping.yaml")

    def _send(self, cmd) -> None:
        """Enqueue a command for the worker (no-op if no worker)."""
        self._cmd_queue.put(cmd)

    # ----------------------------------------------------- Tk refresh loop
    def _on_refresh(self) -> None:
        snap = self.mailbox.get()
        if snap is not None:
            self.render_snapshot(snap)
        self.log_panel.drain()
        # Reschedule unless we are tearing down.
        if self.root.winfo_exists():
            self.root.after(self._refresh_ms, self._on_refresh)

    # ----------------------------------------------------- rendering entry
    def render_snapshot(self, snapshot: Snapshot) -> None:
        """Render one snapshot onto the canvas (Tk-thread only).

        This is the single entry point the refresh loop calls, and is public so
        the smoke test can drive it directly without a worker thread.
        """
        self._current_snapshot = snapshot
        grid = snapshot.grid

        # (Re)build the map image only when the map changed.
        if grid is not None:
            if (self._view is None
                    or snapshot.map_version != self._map_version
                    or self._map_image is None):
                self._rebuild_map(grid, snapshot.map_version)

        view = self._view
        if view is not None:
            map_canvas.clear_dynamic(self.canvas)
            # Order matters: scan/particles/path under the robot markers.
            map_canvas.draw_scan(self.canvas, view, snapshot.scan_points)
            map_canvas.draw_particles(self.canvas, view, snapshot.particles)
            map_canvas.draw_path(self.canvas, view, snapshot.path)
            map_canvas.draw_goal(self.canvas, view, snapshot.goal)
            if snapshot.gt_pose is not None:
                map_canvas.draw_robot(self.canvas, view, snapshot.gt_pose,
                                      self.cfg.robot.radius, color="#9e9e9e",
                                      tag=map_canvas.TAG_GT)
            if snapshot.pose is not None:
                map_canvas.draw_robot(self.canvas, view, snapshot.pose,
                                      self.cfg.robot.radius, color="#1565c0",
                                      tag=map_canvas.TAG_ROBOT)

        self.controls.set_status(self._status_text(snapshot))

    def _rebuild_map(self, grid, map_version: int) -> None:
        self._view = map_canvas.WorldView(grid, self._zoom)
        self._map_image = map_canvas.render_map_image(grid, self._zoom)
        self._map_version = map_version
        self.canvas.delete(map_canvas.TAG_MAP)
        # Keep a reference on the widget too, belt-and-suspenders vs GC.
        self.canvas.image = self._map_image
        self.canvas.create_image(0, 0, anchor="nw", image=self._map_image,
                                 tags=map_canvas.TAG_MAP)
        self.canvas.tag_lower(map_canvas.TAG_MAP)
        self.canvas.configure(scrollregion=(0, 0, self._view.width_px(),
                                            self._view.height_px()))

    def _status_text(self, snap: Snapshot) -> str:
        pose = snap.pose
        estop = "E-STOP" if snap.estop else "-"
        coll = "COLLISION" if snap.collided else "-"
        return ("%s | %s | pose %.2f,%.2f,%.2f | t=%.1fs | %s | %s"
                % (snap.mode.value, snap.nav_state, pose.x, pose.y, pose.theta,
                   snap.sim_time, estop, coll))

    # ------------------------------------------------------- button actions
    def _action_mode_slam(self) -> None:
        self._send(SetMode(Mode.SLAM))
        self._log.info("gui: requested SLAM mode")

    def _action_mode_nav(self) -> None:
        # Entering NAV from SLAM: save the current SLAM map first.
        snap = self._current_snapshot
        if snap is not None and snap.mode == Mode.SLAM:
            self._send(SaveMap("maps/gui_slam"))
            self._log.info("gui: saving SLAM map before NAV switch")
        self._send(SetMode(Mode.NAV))
        self._log.info("gui: requested NAV mode")

    def _action_save_map(self) -> None:
        self._send(SaveMap("maps/gui_save"))
        self._log.info("gui: save map requested")

    def _action_estop(self) -> None:
        self._send(EStop())
        self._log.warning("gui: E-STOP pressed")

    def _action_resume(self) -> None:
        self._send(Resume())
        self._log.info("gui: resume pressed")

    def _action_manual(self, enabled: bool) -> None:
        self._send(SetManual(bool(enabled)))
        if not enabled:
            self._send(SetTeleop(0.0, 0.0))
        self._log.info("gui: manual %s", "on" if enabled else "off")

    def _action_setpose(self, enabled: bool) -> None:
        self._setpose_drag = None
        self._log.info("gui: set-pose %s", "on" if enabled else "off")

    def _action_speed(self, factor: float) -> None:
        if self._worker is not None:
            self._worker.set_speed_factor(float(factor))

    # --------------------------------------------------------- mouse events
    def _on_canvas_press(self, event) -> None:
        if self._view is None:
            return
        if self.controls.setpose_enabled():
            self._setpose_drag = (self.canvas.canvasx(event.x),
                                  self.canvas.canvasy(event.y))
            return
        # Left-click in NAV mode sets a goal.
        snap = self._current_snapshot
        if snap is not None and snap.mode == Mode.NAV:
            wx, wy = self._view.canvas_to_world(self.canvas.canvasx(event.x),
                                                self.canvas.canvasy(event.y))
            self._send(SetGoal(wx, wy))
            self._log.info("gui: goal set at (%.2f, %.2f)", wx, wy)

    def _on_canvas_drag(self, event) -> None:
        # Drag preview for the pose-set heading line.
        if self._view is None or self._setpose_drag is None:
            return
        x0, y0 = self._setpose_drag
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        self.canvas.delete("setpose_preview")
        self.canvas.create_line(x0, y0, cx, cy, fill="#ffeb3b", width=2,
                                arrow="last", tags="setpose_preview")

    def _on_canvas_release(self, event) -> None:
        if self._view is None or self._setpose_drag is None:
            return
        import math
        x0, y0 = self._setpose_drag
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        self._setpose_drag = None
        self.canvas.delete("setpose_preview")
        wx0, wy0 = self._view.canvas_to_world(x0, y0)
        wx1, wy1 = self._view.canvas_to_world(cx, cy)
        theta = math.atan2(wy1 - wy0, wx1 - wx0)
        self._send(SetInitialPose(wx0, wy0, theta))
        self._log.info("gui: initial pose (%.2f, %.2f, %.2f)", wx0, wy0, theta)

    # ------------------------------------------------------ keyboard teleop
    def _on_key_press(self, event) -> None:
        if not self.controls.manual_enabled():
            return
        v, w = self._key_to_twist(event.keysym)
        if v is not None:
            self._send(SetTeleop(v, w))

    def _on_key_release(self, event) -> None:
        if not self.controls.manual_enabled():
            return
        self._send(SetTeleop(0.0, 0.0))

    def _key_to_twist(self, keysym: str):
        k = keysym.lower()
        if k in ("w", "up"):
            return self._TELEOP_V, 0.0
        if k in ("s", "down"):
            return -self._TELEOP_V, 0.0
        if k in ("a", "left"):
            return 0.0, self._TELEOP_W
        if k in ("d", "right"):
            return 0.0, -self._TELEOP_W
        return None, None

    # ------------------------------------------------------------- teardown
    def _on_close(self) -> None:
        # Tell the worker to stop, drain it, then destroy the window.
        if self._worker is not None:
            self._send(Shutdown())
            self._worker.stop()
            self._worker.join(timeout=5.0)
            self._worker = None
        try:
            self.root.destroy()
        except tk.TclError:  # pragma: no cover - already gone
            pass

    # ----------------------------------------------------------------- run
    def run(self) -> None:
        """Enter the Tk main loop (blocks until the window closes)."""
        self.root.mainloop()
