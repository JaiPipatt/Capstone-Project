import math

import numpy as np
import pytest

from msrr_control import wheel_dock_plan as wp
from msrr_control.wheel_dock import WheelDocker

DT = 0.1
PARENT = (0.0, 0.0, 0.0)


def simulate(d, start_pose, start_q, slot_pose, parent_q=0.0, max_ticks=20000):
    """Drive a WheelDocker with an ideal differential-drive model."""
    docker = WheelDocker(d, slot_pose, max_speed=0.049)
    x, y, yaw = start_pose
    q = start_q                                   # angle of wheel d
    for tick in range(max_ticks):
        v, omega = docker.update((x, y, yaw), q, PARENT, parent_q)
        if docker.done or docker.failed:
            break
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = wp.wrap(yaw + omega * DT)
        wheel_speed = v - omega * wp.WHEEL_SEPARATION / 2 * (1 if d == 'L' else -1)
        q += wheel_speed / wp.WHEEL_RADIUS * DT
    return docker, (x, y, yaw), q


@pytest.mark.parametrize('d, slot, start', [
    ('L', (0.0, -0.107, 0.0), (0.0, -0.321, 0.0)),               # parent's right side
    ('R', (0.0, 0.107, 0.0), (0.0, 0.321, 0.0)),                 # parent's left side
    ('L', (0.0, -0.107, 0.0), (0.0, -0.321, math.pi / 2)),       # start facing another way
])
@pytest.mark.parametrize('start_q', [0.0, 0.7, 2.3])
def test_docks_next_to_parent(d, slot, start, start_q):
    docker, pose, q = simulate(d, start, start_q, slot)

    assert docker.done, docker.message
    goal = wp.goal_from_partner(slot, 0.0, d)
    assert np.linalg.norm(wp.wheel_centre(pose, d) - goal.p_f) < 0.005
    assert abs(wp.wrap(pose[2] - goal.yaw_f)) < 0.05
    assert abs(wp.wrap_step(q - goal.q_f)) < 0.2


def test_plan_never_touches_parent():
    goal = wp.goal_from_partner((0.0, -0.107, 0.0), 0.0, 'L')
    plan = wp.plan((0.0, -0.321, 0.0), 0.0, 'L', goal, PARENT)
    assert plan is not None
    assert wp.straight_is_clear(plan.p0, plan.p1, plan.yaw1, 'L', PARENT)
    assert wp.straight_is_clear(plan.p1, plan.p_f, plan.yaw_f, 'L', PARENT)


def test_no_plan_when_start_is_too_close():
    goal = wp.goal_from_partner((0.0, -0.107, 0.0), 0.0, 'L')
    assert wp.plan((0.0, -0.12, 0.0), 0.0, 'L', goal, PARENT) is None
