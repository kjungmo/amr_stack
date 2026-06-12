"""Control + log panel widgets for the HRI console.

These widgets only build and own Tk widgets; they hold no AmrApp reference and
never touch the worker thread directly. Instead they call back into the owning
``AmrGuiApp`` (passed callables), which is the only place that enqueues
commands. Everything here runs on the Tk main thread.

The log panel is fed by :class:`QueueLogHandler`, a ``logging.Handler`` that
pushes formatted records onto a thread-safe ``queue.Queue`` (the worker thread
and any subsystem may log from any thread); the Tk loop drains that queue.
"""
from __future__ import annotations

import logging
import queue
from typing import Callable, Optional

import tkinter as tk
from tkinter import ttk


# ---------------------------------------------------------------------------
# Log plumbing
# ---------------------------------------------------------------------------

class QueueLogHandler(logging.Handler):
    """A logging.Handler that enqueues (levelno, formatted) onto a Queue.

    The Tk loop drains the queue; the handler itself does no Tk work, so it is
    safe to attach to the package logger and have records arrive from worker /
    subsystem threads.
    """

    def __init__(self, log_queue: "queue.Queue", level: int = logging.NOTSET):
        super().__init__(level)
        self._queue = log_queue
        self.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)-7s %(name)s: %(message)s", "%H:%M:%S"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
        except Exception:  # pragma: no cover - never let logging crash the app
            self.handleError(record)
            return
        try:
            self._queue.put_nowait((record.levelno, msg))
        except queue.Full:  # pragma: no cover - unbounded queue
            pass


class LogPanel:
    """A read-only ``tk.Text`` log view with WARN/ERROR coloring.

    ``drain`` is called from the Tk loop: it pops up to ``max_per_drain``
    records off the queue, appends them, and trims to ``max_lines``.
    """

    def __init__(self, parent: tk.Widget, log_queue: "queue.Queue",
                 max_lines: int = 500, max_per_drain: int = 50):
        self._queue = log_queue
        self._max_lines = int(max_lines)
        self._max_per_drain = int(max_per_drain)

        frame = ttk.LabelFrame(parent, text="Log")
        self.frame = frame
        self.text = tk.Text(frame, height=10, width=60, state="disabled",
                            wrap="none", background="#101418",
                            foreground="#d7dde3", font=("TkFixedFont", 8))
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.text.yview)
        self.text.configure(yscrollcommand=scroll.set)
        self.text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

        self.text.tag_configure("WARNING", foreground="#ffb300")
        self.text.tag_configure("ERROR", foreground="#ff5252")
        self.text.tag_configure("CRITICAL", foreground="#ff5252")

    def drain(self) -> None:
        appended = 0
        while appended < self._max_per_drain:
            try:
                levelno, msg = self._queue.get_nowait()
            except queue.Empty:
                break
            self._append(levelno, msg)
            appended += 1
        if appended:
            self._trim()
            self.text.see("end")

    def _append(self, levelno: int, msg: str) -> None:
        tag = logging.getLevelName(levelno)
        self.text.configure(state="normal")
        if tag in ("WARNING", "ERROR", "CRITICAL"):
            self.text.insert("end", msg + "\n", tag)
        else:
            self.text.insert("end", msg + "\n")
        self.text.configure(state="disabled")

    def _trim(self) -> None:
        # Count of lines; tk reports one extra for the trailing newline.
        line_count = int(self.text.index("end-1c").split(".")[0])
        if line_count > self._max_lines:
            self.text.configure(state="normal")
            self.text.delete("1.0", "%d.0" % (line_count - self._max_lines + 1))
            self.text.configure(state="disabled")


# ---------------------------------------------------------------------------
# Control panel
# ---------------------------------------------------------------------------

class ControlPanel:
    """Mode + action buttons, speed scale, and a status bar.

    All buttons invoke callbacks supplied by ``AmrGuiApp``; the panel owns no
    runtime state beyond the Tk variables backing the manual toggle, the
    "set pose" toggle, and the speed scale.
    """

    def __init__(self, parent: tk.Widget,
                 on_set_mode_slam: Callable[[], None],
                 on_set_mode_nav: Callable[[], None],
                 on_save_map: Callable[[], None],
                 on_estop: Callable[[], None],
                 on_resume: Callable[[], None],
                 on_manual_toggle: Callable[[bool], None],
                 on_setpose_toggle: Callable[[bool], None],
                 on_speed: Callable[[float], None],
                 initial_speed: float = 1.0):
        frame = ttk.Frame(parent, padding=6)
        self.frame = frame

        # --- Mode buttons -------------------------------------------------
        mode_box = ttk.LabelFrame(frame, text="Mode")
        mode_box.pack(fill="x", pady=(0, 6))
        ttk.Button(mode_box, text="SLAM Mapping",
                   command=on_set_mode_slam).pack(side="left", padx=3, pady=3)
        ttk.Button(mode_box, text="Navigate",
                   command=on_set_mode_nav).pack(side="left", padx=3, pady=3)

        # --- Action buttons ----------------------------------------------
        act_box = ttk.LabelFrame(frame, text="Actions")
        act_box.pack(fill="x", pady=(0, 6))
        ttk.Button(act_box, text="Save Map",
                   command=on_save_map).pack(side="left", padx=3, pady=3)
        self.estop_btn = tk.Button(act_box, text="E-STOP", fg="white",
                                   bg="#c62828", activebackground="#b71c1c",
                                   command=on_estop)
        self.estop_btn.pack(side="left", padx=3, pady=3)
        ttk.Button(act_box, text="Resume",
                   command=on_resume).pack(side="left", padx=3, pady=3)

        # --- Toggles ------------------------------------------------------
        tog_box = ttk.LabelFrame(frame, text="Teleop / Pose")
        tog_box.pack(fill="x", pady=(0, 6))
        self.manual_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tog_box, text="Manual", variable=self.manual_var,
                        command=lambda: on_manual_toggle(self.manual_var.get())
                        ).pack(side="left", padx=3, pady=3)
        self.setpose_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tog_box, text="Set Pose", variable=self.setpose_var,
                        command=lambda: on_setpose_toggle(self.setpose_var.get())
                        ).pack(side="left", padx=3, pady=3)

        # --- Speed scale --------------------------------------------------
        speed_box = ttk.LabelFrame(frame, text="Speed x")
        speed_box.pack(fill="x", pady=(0, 6))
        self.speed_var = tk.DoubleVar(value=float(initial_speed))
        self.speed_scale = ttk.Scale(
            speed_box, from_=0.5, to=4.0, orient="horizontal",
            variable=self.speed_var,
            command=lambda _v: on_speed(self.speed_var.get()))
        self.speed_scale.pack(fill="x", padx=4, pady=4)

        # --- Status bar ---------------------------------------------------
        self.status_var = tk.StringVar(value="(starting)")
        status = ttk.Label(frame, textvariable=self.status_var, anchor="w",
                           relief="sunken", padding=3)
        status.pack(fill="x", side="bottom")

    # ----------------------------------------------------------- updaters
    def set_status(self, text: str) -> None:
        self.status_var.set(text)

    def manual_enabled(self) -> bool:
        return bool(self.manual_var.get())

    def setpose_enabled(self) -> bool:
        return bool(self.setpose_var.get())
