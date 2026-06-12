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
