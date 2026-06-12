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
