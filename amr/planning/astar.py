"""A* global planner over an inflated costmap, plus a line-of-sight helper.

8-connected heapq search on grid cells. Lethal cells are obstacles; the step
cost is scaled up by the (non-lethal) cell cost so the path is pushed away from
obstacles. The heuristic is the octile distance, which is admissible because the
step-cost multiplier ``1 + w_cost * cost`` is always >= 1.
"""
from __future__ import annotations

import heapq
import math
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

from amr.core.config import AstarConfig
from amr.planning.costmap import Costmap

_SQRT2 = math.sqrt(2.0)

# 8-connected neighbour offsets with their step lengths (cell units).
_NEIGHBORS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2),
]


def _octile(r0: int, c0: int, r1: int, c1: int) -> float:
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    lo, hi = (dr, dc) if dr < dc else (dc, dr)
    return (hi - lo) + _SQRT2 * lo


def _nearest_non_lethal(costmap: Costmap, row: int, col: int,
                        max_dist_m: float) -> Optional[Tuple[int, int]]:
    """BFS outward from (row, col) for the nearest non-lethal cell within
    ``max_dist_m`` metres. Returns the cell or None."""
    if costmap.in_bounds(row, col) and not costmap.is_lethal(row, col):
        return (row, col)
    max_cells = int(math.floor(max_dist_m / costmap.resolution))
    seen = {(row, col)}
    q = deque([(row, col, 0)])
    while q:
        r, c, d = q.popleft()
        if d >= max_cells:
            continue
        for dr, dc, _ in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if (nr, nc) in seen:
                continue
            seen.add((nr, nc))
            if not costmap.in_bounds(nr, nc):
                continue
            if not costmap.is_lethal(nr, nc):
                return (nr, nc)
            q.append((nr, nc, d + 1))
    return None


def has_line_of_sight(costmap: Costmap, p, q) -> bool:
    """True iff every sample at resolution/2 spacing along p->q is non-lethal."""
    px, py = float(p[0]), float(p[1])
    qx, qy = float(q[0]), float(q[1])
    dist = math.hypot(qx - px, qy - py)
    step = costmap.resolution * 0.5
    n = int(math.ceil(dist / step)) if dist > 0 else 0
    for i in range(n + 1):
        t = (i / n) if n > 0 else 0.0
        x = px + t * (qx - px)
        y = py + t * (qy - py)
        if costmap.cost_at_world(x, y) >= 0.99:
            return False
    return True


def _simplify(costmap: Costmap, pts: List[Tuple[float, float]]
              ) -> List[Tuple[float, float]]:
    """Greedy shortcutting: drop a point when there is line of sight from the
    previous kept point to the point after it."""
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = i + 1
        # Extend as far as line of sight from out[-1] permits.
        while j + 1 < len(pts) and has_line_of_sight(costmap, out[-1], pts[j + 1]):
            j += 1
        out.append(pts[j])
        i = j
    return out


def plan_path(costmap: Costmap, start_xy, goal_xy,
              cfg: AstarConfig) -> Optional[np.ndarray]:
    """Plan an 8-connected A* path; return (M, 2) world waypoints start->goal,
    or None if unreachable."""
    start_x, start_y = float(start_xy[0]), float(start_xy[1])
    goal_x, goal_y = float(goal_xy[0]), float(goal_xy[1])

    sr, sc = costmap.world_to_grid(start_x, start_y)
    gr, gc = costmap.world_to_grid(goal_x, goal_y)

    if not costmap.in_bounds(gr, gc) or costmap.is_lethal(gr, gc):
        return None

    # If the start cell is lethal, search outward (<= 0.3 m) for a free cell.
    start_cell = _nearest_non_lethal(costmap, sr, sc, 0.3)
    if start_cell is None:
        return None
    sr, sc = start_cell

    if (sr, sc) == (gr, gc):
        return np.array([[start_x, start_y], [goal_x, goal_y]], dtype=float)

    cost = costmap.cost
    w_cost = cfg.w_cost

    open_heap = [(_octile(sr, sc, gr, gc), 0.0, sr, sc)]
    g_score = {(sr, sc): 0.0}
    came_from = {}
    closed = set()

    found = False
    while open_heap:
        _, g_cur, r, c = heapq.heappop(open_heap)
        if (r, c) in closed:
            continue
        if (r, c) == (gr, gc):
            found = True
            break
        closed.add((r, c))
        for dr, dc, step_len in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if not costmap.in_bounds(nr, nc):
                continue
            if (nr, nc) in closed:
                continue
            if cost[nr, nc] >= 0.99:        # lethal cell excluded
                continue
            tentative = g_cur + step_len * (1.0 + w_cost * float(cost[nr, nc]))
            prev = g_score.get((nr, nc))
            if prev is None or tentative < prev:
                g_score[(nr, nc)] = tentative
                came_from[(nr, nc)] = (r, c)
                f = tentative + _octile(nr, nc, gr, gc)
                heapq.heappush(open_heap, (f, tentative, nr, nc))

    if not found:
        return None

    # Reconstruct cell path goal -> start, then reverse to start -> goal.
    cells: List[Tuple[int, int]] = [(gr, gc)]
    node = (gr, gc)
    while node in came_from:
        node = came_from[node]
        cells.append(node)
    cells.reverse()

    # World points at cell centers, with exact start prepended and exact goal
    # appended.
    pts: List[Tuple[float, float]] = [(start_x, start_y)]
    for (r, c) in cells:
        wx, wy = costmap.grid_to_world(r, c)
        pts.append((wx, wy))
    pts.append((goal_x, goal_y))

    if cfg.simplify:
        pts = _simplify(costmap, pts)

    return np.array(pts, dtype=float)
