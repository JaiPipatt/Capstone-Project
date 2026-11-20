"""
Plan for docking a module's wheel face (L or R) onto a parent module.

The plan is four moves:  pivot -> straight -> pivot -> straight
  pivot 1     hold wheel d still, turn the body to face point p1
  straight 1  drive the wheel from p0 to p1
  pivot 2     hold wheel d still, turn the body to the final heading
  straight 2  drive from p1 to the goal p_f along the parent's face

The EP face has 4-fold symmetry, so the docking wheel only has to end at the
right angle modulo pi/2. That picks the distance s between p1 and p_f.

Conventions
  pose = (x, y, yaw), x/y is the body centre (the axle midpoint), yaw is the
         direction the module drives forward. Left of the module is yaw + 90 deg.
  d    = the docking wheel, 'L' or 'R'.
  q    = joint angle of wheel d. It grows when the module drives forward.
No ROS in this file.
"""
import math
from dataclasses import dataclass

import numpy as np

# ------------------------------------------------------------------ constants
WHEEL_RADIUS = 0.05          # r
WHEEL_SEPARATION = 0.09665   # W
BODY = 0.107                 # B, side of the square body
STEP = math.pi / 2           # face symmetry: wheel angle only matters mod pi/2
R_MAX = math.hypot(BODY / 2 + WHEEL_SEPARATION / 2, BODY / 2)   # farthest body point from a wheel
S_MIN = max(2 * WHEEL_RADIUS, R_MAX + BODY / 2)                 # shortest allowed last straight
S_MAX = 0.5                  # longest last straight searched
FACE_GAP = 0.003             # stop this far from the parent, magnets close the rest
C_MIRROR = 0.0               # q_goal = C_MIRROR - q of the parent's face. Calibrate by hand.
M_SAFE = 0.01                # clearance wanted before cost matters


# ------------------------------------------------------------------ small geometry helpers
def wrap(angle):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_step(angle):
    """Distance of angle from the nearest multiple of pi/2."""
    return angle - STEP * round(angle / STEP)


def forward(yaw):
    return np.array([math.cos(yaw), math.sin(yaw)])


def left(yaw):
    return np.array([-math.sin(yaw), math.cos(yaw)])


def yaw_of(vector):
    return math.atan2(vector[1], vector[0])


def outward(yaw, d):
    """Unit vector from the body centre towards wheel d."""
    return left(yaw) if d == 'L' else -left(yaw)


def wheel_centre(pose, d):
    return np.array(pose[:2]) + outward(pose[2], d) * WHEEL_SEPARATION / 2


def body_centre(wheel_pos, yaw, d):
    return wheel_pos - outward(yaw, d) * WHEEL_SEPARATION / 2


# ------------------------------------------------------------------ the parent (a square)
def square_corners(centre, yaw):
    f, l = forward(yaw) * BODY / 2, left(yaw) * BODY / 2
    return [centre + f + l, centre + f - l, centre - f - l, centre - f + l]


def squares_overlap(centre_a, yaw_a, centre_b, yaw_b):
    """Separating axis test for two body squares."""
    corners_a = square_corners(centre_a, yaw_a)
    corners_b = square_corners(centre_b, yaw_b)
    for yaw in (yaw_a, yaw_b):
        for axis in (forward(yaw), left(yaw)):
            a = [c @ axis for c in corners_a]
            b = [c @ axis for c in corners_b]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


def distance_to_square(point, parent_pose):
    """Distance from a point to the parent's body square (0 if inside)."""
    rel = np.array(point) - np.array(parent_pose[:2])
    x = abs(rel @ forward(parent_pose[2])) - BODY / 2
    y = abs(rel @ left(parent_pose[2])) - BODY / 2
    return math.hypot(max(x, 0.0), max(y, 0.0))


def straight_is_clear(wheel_from, wheel_to, yaw, d, parent_pose):
    """Does the body hit the parent while the wheel drives from wheel_from to wheel_to?"""
    parent = np.array(parent_pose[:2])
    n = int(np.linalg.norm(wheel_to - wheel_from) / 0.01) + 2
    for t in np.linspace(0.0, 1.0, n):
        wheel = wheel_from + (wheel_to - wheel_from) * t
        if squares_overlap(body_centre(wheel, yaw, d), yaw, parent, parent_pose[2]):
            return False
    return True


# ------------------------------------------------------------------ goal and plan
@dataclass
class Goal:
    p_f: np.ndarray   # where wheel d must end
    yaw_f: float      # heading at the end
    q_f: float        # wheel d angle at the end (mod pi/2)


def goal_from_partner(slot_pose, parent_face_q, d):
    """slot_pose: where this module's body ends. parent_face_q: wheel angle of the parent's face."""
    # the docking wheel faces the parent, so stop a little short of it
    p_f = wheel_centre(slot_pose, d) - outward(slot_pose[2], d) * FACE_GAP
    return Goal(p_f, slot_pose[2], C_MIRROR - parent_face_q)


@dataclass
class Plan:
    p0: np.ndarray
    p1: np.ndarray
    p_f: np.ndarray
    yaw1: float       # heading during straight 1
    yaw_f: float      # heading during straight 2
    dist1: float      # signed distance of straight 1 (negative = reverse)
    dist2: float      # signed distance of straight 2
    cost: float
    margin: float


def find_roots(f, lo, hi, n=500):
    """Roots of f in [lo, hi] by sign change and bisection."""
    roots = []
    xs = np.linspace(lo, hi, n)
    for a, b in zip(xs[:-1], xs[1:]):
        if f(a) * f(b) < 0:
            for _ in range(40):
                mid = (a + b) / 2
                a, b = (mid, b) if f(a) * f(mid) > 0 else (a, mid)
            roots.append(a)
    return roots


def plan(pose, q, d, goal, parent_pose):
    """Best plan from the current pose, or None."""
    p0 = wheel_centre(pose, d)
    candidates = []

    for dir01 in (+1, -1):          # +1 drive straight 1 forwards, -1 in reverse
        def angle_error(s):
            # wheel angle at the end of the plan minus wanted, 4-fold: zero when it matches
            p1 = goal.p_f + s * forward(goal.yaw_f)
            dist1 = dir01 * np.linalg.norm(p1 - p0)
            dist2 = -s
            return math.sin(2 * (q + (dist1 + dist2) / WHEEL_RADIUS - goal.q_f))

        for sign in (+1, -1):
            for s in find_roots(angle_error, sign * S_MIN, sign * S_MAX):
                candidate = _make_candidate(pose, p0, s, dir01, goal, d, parent_pose)
                if candidate:
                    candidates.append(candidate)

    if not candidates:
        return None
    safe = [c for c in candidates if c.margin >= M_SAFE]
    if safe:
        return min(safe, key=lambda c: c.cost)
    return max(candidates, key=lambda c: c.margin)


def _make_candidate(pose, p0, s, dir01, goal, d, parent_pose):
    p1 = goal.p_f + s * forward(goal.yaw_f)
    if np.linalg.norm(p1 - p0) < 0.01:
        return None                     # no direction to drive in

    yaw1 = yaw_of(dir01 * (p1 - p0))    # heading while driving p0 -> p1
    turn1 = wrap(yaw1 - pose[2])
    turn2 = wrap(goal.yaw_f - yaw1)

    # clearance: pivots must not swing the body into the parent, straights must not touch it
    clear0 = distance_to_square(p0, parent_pose) - R_MAX
    clear1 = distance_to_square(p1, parent_pose) - R_MAX
    if min(clear0, clear1) < -1e-6:
        return None
    if not straight_is_clear(p0, p1, yaw1, d, parent_pose):
        return None
    if not straight_is_clear(p1, goal.p_f, goal.yaw_f, d, parent_pose):
        return None

    length = np.linalg.norm(p1 - p0) + abs(s)
    cost = math.sqrt(2) * length + (abs(turn1) + abs(turn2)) * WHEEL_SEPARATION
    margin = min(clear0, clear1, abs(s) - 2 * WHEEL_RADIUS)
    return Plan(p0, p1, goal.p_f, yaw1, goal.yaw_f,
                dir01 * np.linalg.norm(p1 - p0), -s, cost, margin)
