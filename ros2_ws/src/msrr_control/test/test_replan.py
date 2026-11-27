from msrr_control.navigation import Grid
from msrr_control.replan import off_track, Replanner, StuckDetector

CELL = 0.107
GRID = Grid((0.0, 0.0, 0.0), CELL, 5)
SLOTS = {0: (0.0, 0.0, 0.0), 1: (CELL, 0.0, 0.0)}


def test_not_stuck_while_moving_or_turning():
    det = StuckDetector(timeout=2.0)
    for k in range(50):
        assert not det.update('m', (0.02 * k, 0.0, 0.0), 0.1 * k)
    for k in range(50):
        assert not det.update('m', (1.0, 0.0, 0.2 * k), 5.0 + 0.1 * k)


def test_stuck_after_timeout_and_reset():
    det = StuckDetector(timeout=2.0)
    det.update('m', (0.0, 0.0, 0.0), 0.0)
    assert not det.update('m', (0.001, 0.0, 0.0), 1.9)
    assert det.update('m', (0.002, 0.0, 0.0), 2.1)
    det.reset('m')
    assert not det.update('m', (0.002, 0.0, 0.0), 2.2)


def test_replan_avoids_stuck_node_and_other_modules():
    rp = Replanner(GRID, SLOTS)
    pose = (0.0, -3 * CELL, 0.0)
    pre_dock = (0.0, 3 * CELL, 0.0)
    stuck_target = (-CELL, -CELL, 0.0)
    other = (-CELL, 0.0, 0.0)
    route = rp.replan('m', pose, pre_dock, stuck_target, {0, 1}, [other])
    nodes = [GRID.to_node(p) for p in route[1:-1]]
    assert route[0] == pose[:2] and route[-1] == pre_dock[:2]
    for bad in [(0, 0), (1, 0), (-1, -1), (-1, 0)]:
        assert bad not in nodes
    for a, b in zip(nodes, nodes[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1


def test_falls_back_when_only_path_is_through_avoided_node():
    grid = Grid((0.0, 0.0, 0.0), CELL, 1)          # 3x3 grid
    rp = Replanner(grid, {0: (0.0, 0.0, 0.0)})
    # root blocks the centre; the other module sits on the only remaining side
    route = rp.replan('m', (-CELL, -CELL, 0.0), (CELL, CELL, 0.0), (CELL, -CELL, 0.0),
                      {0}, [(-CELL, CELL, 0.0)])
    assert route is not None


def test_gives_up_after_max_replans():
    rp = Replanner(GRID, SLOTS, max_replans=2)
    args = ('m', (0.0, -2 * CELL, 0.0), (0.0, 2 * CELL, 0.0), (0.0, -CELL, 0.0), {0}, [])
    assert rp.replan(*args) is not None
    assert rp.replan(*args) is not None
    assert rp.replan(*args) is None


def test_off_track_distance_to_segment():
    a, b = (0.0, 0.0), (CELL, 0.0)
    assert off_track((0.05, 0.02, 0.0), a, b, 0.03) is None       # small sideways error
    assert abs(off_track((0.05, 0.04, 0.0), a, b, 0.03) - 0.04) < 1e-9
    assert off_track((CELL + 0.02, 0.0, 0.0), a, b, 0.03) is None  # just past the end, on line
    assert off_track((CELL, 0.0, 0.0), a, a, 0.03) is not None     # zero-length segment


def test_off_track_replan_does_not_block_target_and_has_own_budget():
    rp = Replanner(GRID, SLOTS, max_replans=1, max_offtrack_replans=2)
    pose = (0.0, -2 * CELL + 0.04, 0.0)
    pre_dock = (0.0, -CELL, 0.0)
    for _ in range(2):
        route = rp.replan('m', pose, pre_dock, None, {0}, [], 'off_track')
        assert route is not None
        assert [GRID.to_node(p) for p in route[1:-1]] == [(0, -2), (0, -1)]
    assert rp.replan('m', pose, pre_dock, None, {0}, [], 'off_track') is None
    assert rp.replan('m', pose, pre_dock, (0.0, -CELL), {0}, [], 'stuck') is not None
