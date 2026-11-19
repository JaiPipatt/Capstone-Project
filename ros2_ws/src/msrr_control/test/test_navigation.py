import math

from msrr_control.navigation import astar, Grid, make_grid
import pytest


def test_grid_round_trip_in_rotated_frame():
    grid = Grid((1.0, 2.0, math.pi / 2), 0.1, 5)
    assert grid.to_node(grid.to_world((3, -2))) == (3, -2)
    # +i follows the root's heading, here world +y
    assert grid.to_world((1, 0)) == pytest.approx((1.0, 2.1))


def test_make_grid_covers_points():
    grid = make_grid((0.0, 0.0, 0.0), 0.1, [(0.42, -0.1), (-0.2, 0.3)], margin=1)
    assert grid.half_size == 5


def test_astar_straight_line():
    path = astar(Grid((0, 0, 0), 0.1, 5), (0, 0), (3, 0))
    assert path == [(0, 0), (1, 0), (2, 0), (3, 0)]


def test_astar_goes_around_blocked_node():
    path = astar(Grid((0, 0, 0), 0.1, 5), (-1, 0), (1, 0), blocked={(0, 0)})
    assert (0, 0) not in path
    assert len(path) == 5  # shortest detour: up, across two, down


def test_astar_no_path():
    with pytest.raises(ValueError):
        astar(Grid((0, 0, 0), 0.1, 1), (-1, 0), (1, 0),
              blocked={(0, -1), (0, 0), (0, 1)})
