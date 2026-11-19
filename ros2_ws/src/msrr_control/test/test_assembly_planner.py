import math

from msrr_control.assembly import assign_modules_to_slots, parallel_assembly
from msrr_control.topology import check_topology, find_root, Topology, unfold
import pytest


def chain(n):
    return Topology(n, [(i, 'R', i + 1, 'L') for i in range(n - 1)])


def test_root_of_chain_of_5_is_middle():
    root, _ = find_root(chain(5))
    assert root == 2


def test_chain_of_4_counts_and_root():
    root, cn = find_root(chain(4), v0=0)
    assert root in (1, 2)
    assert (cn[2]['R'], cn[3]['L']) == (1, 3)
    assert (cn[1]['R'], cn[2]['L']) == (2, 2)
    assert (cn[0]['R'], cn[1]['L']) == (3, 1)


def test_root_does_not_depend_on_v0():
    for v0 in range(5):
        assert find_root(chain(5), v0)[0] == 2


def test_top_top_child_pose():
    slots = unfold(Topology(2, [(0, 'T', 1, 'T')]), 0, 0.08)
    x, y, th = slots[1]
    assert (x, y) == pytest.approx((0.08, 0.0))
    assert abs(th) == pytest.approx(math.pi)


def test_cross_with_tail_rounds():
    g = Topology(5, [(1, 'B', 0, 'L'), (2, 'B', 0, 'R'), (3, 'T', 0, 'B'), (4, 'T', 3, 'B')])
    f = {0: 'R2', 1: 'R0', 2: 'R4', 3: 'R1', 4: 'R3'}
    assert parallel_assembly(g, 0, f) == [
        [('R0', 'B', 'R2', 'L'), ('R4', 'B', 'R2', 'R')],
        [('R1', 'T', 'R2', 'B')],
        [('R3', 'T', 'R1', 'B')],
    ]


def test_loop_is_rejected():
    with pytest.raises(ValueError):
        check_topology(Topology(4, [(0, 'T', 1, 'B'), (1, 'L', 2, 'R'),
                                  (2, 'B', 3, 'T'), (3, 'R', 0, 'L')]), 0.1)


def test_l_shape_is_accepted():
    check_topology(Topology(3, [(1, 'B', 0, 'T'), (2, 'B', 0, 'L')]), 0.1)


def test_assign_maps_nearest_module_to_each_slot():
    slots = unfold(Topology(3, [(1, 'B', 0, 'T'), (2, 'B', 0, 'L')]), 0, 0.1)
    modules = {'a': (0.0, 0.0, 0.0), 'b': (0.0, 0.5, 0.0), 'c': (0.5, 0.0, 0.0)}
    f, m_root = assign_modules_to_slots(slots, 0, modules)
    assert m_root == 'a'
    assert f == {0: 'a', 1: 'c', 2: 'b'}