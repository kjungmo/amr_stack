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
