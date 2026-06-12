"""ROS-compatible map I/O: save/load PGM+YAML map file pairs.

Pixel convention (plan §2 item 6):
  occupied (100)  → pixel 0
  free     (0)    → pixel 254
  unknown  (-1)   → pixel 205

Image rows are flipped (np.flipud) so that image row 0 = north (top of screen)
while grid row 0 = south (minimum y). Load reverses the flip.
"""
from __future__ import annotations

import os
import struct
from typing import Tuple

import numpy as np
import yaml

from amr.core.types import OccupancyGrid


def save_map(grid: OccupancyGrid, path_stem: str) -> Tuple[str, str]:
    """Write *path_stem*.pgm and *path_stem*.yaml.

    Returns (pgm_path, yaml_path).
    """
    pgm_path = path_stem + ".pgm"
    yaml_path = path_stem + ".yaml"

    rows, cols = grid.data.shape

    # Map int8 occupancy values to PGM pixel values
    pixels = np.empty((rows, cols), dtype=np.uint8)
    pixels[grid.data == 100] = 0      # occupied  → black
    pixels[grid.data == 0] = 254      # free       → white
    pixels[grid.data == -1] = 205     # unknown    → grey
    # Any other values (shouldn't occur) default to unknown
    mask_other = (grid.data != 100) & (grid.data != 0) & (grid.data != -1)
    pixels[mask_other] = 205

    # Flip so image row 0 = top (north)
    pixels_img = np.flipud(pixels)

    # Write PGM P5 (binary)
    with open(pgm_path, "wb") as f:
        header = "P5\n{} {}\n255\n".format(cols, rows).encode("ascii")
        f.write(header)
        f.write(pixels_img.tobytes())

    # Write YAML sidecar
    pgm_basename = os.path.basename(pgm_path)
    meta = {
        "image": pgm_basename,
        "resolution": float(grid.resolution),
        "origin": [float(grid.origin_x), float(grid.origin_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(yaml_path, "w") as f:
        yaml.dump(meta, f, default_flow_style=False)

    return pgm_path, yaml_path


def load_map(yaml_path: str) -> OccupancyGrid:
    """Load a PGM+YAML map pair.  The PGM is looked up relative to the YAML."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    resolution = float(meta["resolution"])
    origin = meta["origin"]
    origin_x = float(origin[0])
    origin_y = float(origin[1])

    # Resolve PGM path relative to the YAML file's directory
    pgm_name = meta["image"]
    if not os.path.isabs(pgm_name):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_name)
    else:
        pgm_path = pgm_name

    # Parse PGM P5
    with open(pgm_path, "rb") as f:
        raw = f.read()

    # Parse header: "P5\n<cols> <rows>\n<maxval>\n"
    pos = 0

    def _next_token():
        nonlocal pos
        # skip whitespace and comments
        while pos < len(raw):
            if raw[pos:pos + 1] == b"#":
                while pos < len(raw) and raw[pos:pos + 1] != b"\n":
                    pos += 1
            elif raw[pos:pos + 1] in (b" ", b"\t", b"\r", b"\n"):
                pos += 1
            else:
                break
        start = pos
        while pos < len(raw) and raw[pos:pos + 1] not in (b" ", b"\t", b"\r", b"\n"):
            pos += 1
        return raw[start:pos].decode("ascii")

    magic = _next_token()
    assert magic == "P5", "Expected P5 PGM, got {}".format(magic)
    cols = int(_next_token())
    rows = int(_next_token())
    _maxval = int(_next_token())
    # After the maxval, exactly one whitespace byte separates the header from data
    pos += 1  # skip single whitespace delimiter

    pixel_bytes = raw[pos:pos + rows * cols]
    pixels_img = np.frombuffer(pixel_bytes, dtype=np.uint8).reshape(rows, cols)

    # Reverse the flip applied on save
    pixels = np.flipud(pixels_img)

    # Map pixel values back to ROS occupancy.
    # Pixel convention: occupied=0, unknown=205, free=254.
    # Use probability-based thresholds consistent with occupied_thresh=0.65,
    # free_thresh=0.25 when p = (255-pixel)/255:
    #   occupied: p > 0.65  ↔ pixel < 255*(1-0.65) = 89.25  → pixel ≤ 89
    #   free:     p < 0.25  but only for true-free pixels (254); unknown (205)
    #             must remain -1. Use strict threshold: pixel >= 250.
    data = np.full((rows, cols), -1, dtype=np.int8)
    data[pixels <= 50] = 100     # dark → occupied  (covers 0)
    data[pixels >= 250] = 0      # very bright → free  (covers 254, not 205)
    # 51..249 → unknown (-1) already set

    return OccupancyGrid(resolution=resolution, origin_x=origin_x,
                         origin_y=origin_y, data=data)
