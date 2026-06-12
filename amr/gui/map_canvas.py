"""Canvas rendering for the HRI console.

This module owns the *pure drawing* concerns of the GUI: converting an
:class:`~amr.core.types.OccupancyGrid` into a ``tk.PhotoImage`` (no PIL), the
world<->pixel coordinate transforms, and the per-refresh overlay drawing
helpers (robot, lidar, particles, path, goal). Every helper here only touches
Tk widgets, so it must be called from the Tk main thread.

Coordinate / data conventions follow plan §2. The world->pixel transform is::

    cx = (x - origin_x) / res * zoom
    cy = (rows - (y - origin_y) / res) * zoom

with ``canvas_to_world`` the exact inverse (used for mouse events).
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import tkinter as tk

from amr.core.types import OccupancyGrid, Pose2D


# ---------------------------------------------------------------------------
# Map image
# ---------------------------------------------------------------------------

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


def grid_to_photoimage_p6(grid, zoom):
    """P6 (RGB) fallback if a Tk build rejects the P5 (grayscale) PGM.

    Repeats the gray channel three times so the same byte stream is decoded as
    color. Used only when :func:`grid_to_photoimage` raises ``tk.TclError``.
    """
    g = np.full(grid.data.shape, 128, dtype=np.uint8)
    g[grid.data == 0] = 230
    g[grid.data >= 65] = 30
    g = np.flipud(g)
    rgb = np.repeat(g[:, :, None], 3, axis=2)
    h, w = g.shape
    header = ("P6 %d %d 255 " % (w, h)).encode("ascii")
    img = tk.PhotoImage(data=header + rgb.tobytes(), format="PPM")
    return img.zoom(zoom, zoom)


def render_map_image(grid, zoom):
    """Build the map PhotoImage, falling back from P5 to P6 on TclError."""
    try:
        return grid_to_photoimage(grid, zoom)
    except tk.TclError:
        return grid_to_photoimage_p6(grid, zoom)


# ---------------------------------------------------------------------------
# World <-> pixel transforms
# ---------------------------------------------------------------------------

class WorldView:
    """Caches the parameters of the current map for the world<->pixel maps.

    ``rows`` is the grid row count; the y-axis is flipped so that increasing
    world-y goes *up* on the canvas (image row 0 = top).
    """

    def __init__(self, grid: OccupancyGrid, zoom: int):
        self.res = float(grid.resolution)
        self.origin_x = float(grid.origin_x)
        self.origin_y = float(grid.origin_y)
        self.rows = int(grid.rows)
        self.cols = int(grid.cols)
        self.zoom = int(zoom)

    def world_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        cx = (x - self.origin_x) / self.res * self.zoom
        cy = (self.rows - (y - self.origin_y) / self.res) * self.zoom
        return cx, cy

    def canvas_to_world(self, cx: float, cy: float) -> Tuple[float, float]:
        x = cx / self.zoom * self.res + self.origin_x
        y = (self.rows - cy / self.zoom) * self.res + self.origin_y
        return x, y

    def width_px(self) -> int:
        return self.cols * self.zoom

    def height_px(self) -> int:
        return self.rows * self.zoom


# ---------------------------------------------------------------------------
# Overlay drawing (all tagged so a refresh can delete-then-redraw)
# ---------------------------------------------------------------------------

TAG_MAP = "mapimg"
TAG_ROBOT = "robot"
TAG_GT = "gt"
TAG_SCAN = "scan"
TAG_PARTICLES = "particles"
TAG_PATH = "path"
TAG_GOAL = "goal"

# Overlay tags wiped every refresh (the map image is handled separately so it
# is only re-rendered on a map_version change).
DYNAMIC_TAGS = (TAG_ROBOT, TAG_GT, TAG_SCAN, TAG_PARTICLES, TAG_PATH, TAG_GOAL)


def clear_dynamic(canvas: tk.Canvas) -> None:
    for tag in DYNAMIC_TAGS:
        canvas.delete(tag)


def draw_robot(canvas: tk.Canvas, view: WorldView, pose: Pose2D,
               radius_m: float, color: str = "#1565c0",
               tag: str = TAG_ROBOT) -> None:
    """Footprint circle + heading line at ``pose`` (world frame)."""
    cx, cy = view.world_to_canvas(pose.x, pose.y)
    r = max(3.0, radius_m / view.res * view.zoom)
    canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                       outline=color, width=2, tags=tag)
    hx = pose.x + math.cos(pose.theta) * radius_m * 1.6
    hy = pose.y + math.sin(pose.theta) * radius_m * 1.6
    ex, ey = view.world_to_canvas(hx, hy)
    canvas.create_line(cx, cy, ex, ey, fill=color, width=2, tags=tag)


def draw_scan(canvas: tk.Canvas, view: WorldView,
              points: Optional[np.ndarray], limit: int = 120,
              color: str = "#e53935") -> None:
    """Lidar endpoints as small red dots (subsampled to ``limit``)."""
    if points is None:
        return
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 2:
        return
    if pts.shape[0] > limit:
        idx = np.linspace(0, pts.shape[0] - 1, limit).astype(int)
        pts = pts[idx]
    for x, y in pts:
        cx, cy = view.world_to_canvas(float(x), float(y))
        canvas.create_oval(cx - 1.5, cy - 1.5, cx + 1.5, cy + 1.5,
                           fill=color, outline=color, tags=TAG_SCAN)


def draw_particles(canvas: tk.Canvas, view: WorldView,
                   particles: Optional[np.ndarray], limit: int = 300,
                   color: str = "#2e7d32") -> None:
    """Particle cloud as short heading ticks (subsampled to ``limit``)."""
    if particles is None:
        return
    ps = np.asarray(particles, dtype=float)
    if ps.ndim != 2 or ps.shape[0] == 0 or ps.shape[1] < 3:
        return
    if ps.shape[0] > limit:
        idx = np.linspace(0, ps.shape[0] - 1, limit).astype(int)
        ps = ps[idx]
    tick = 0.12  # m
    for x, y, th in ps[:, :3]:
        cx, cy = view.world_to_canvas(float(x), float(y))
        ex, ey = view.world_to_canvas(float(x) + math.cos(th) * tick,
                                      float(y) + math.sin(th) * tick)
        canvas.create_line(cx, cy, ex, ey, fill=color, width=1,
                           tags=TAG_PARTICLES)


def draw_path(canvas: tk.Canvas, view: WorldView,
              path: Optional[np.ndarray], color: str = "#fb8c00") -> None:
    """Global path as an orange polyline."""
    if path is None:
        return
    pp = np.asarray(path, dtype=float)
    if pp.ndim != 2 or pp.shape[0] < 2 or pp.shape[1] < 2:
        return
    coords: List[float] = []
    for x, y in pp[:, :2]:
        cx, cy = view.world_to_canvas(float(x), float(y))
        coords.extend((cx, cy))
    canvas.create_line(*coords, fill=color, width=2, tags=TAG_PATH)


def draw_goal(canvas: tk.Canvas, view: WorldView,
              goal: Optional[Pose2D], color: str = "#6a1b9a") -> None:
    """Goal marker as a cross."""
    if goal is None:
        return
    cx, cy = view.world_to_canvas(goal.x, goal.y)
    s = 7.0
    canvas.create_line(cx - s, cy, cx + s, cy, fill=color, width=2, tags=TAG_GOAL)
    canvas.create_line(cx, cy - s, cx, cy + s, fill=color, width=2, tags=TAG_GOAL)
