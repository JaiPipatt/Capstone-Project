"""
Stuck / off-track detection and grid replanning for the navigate phase.

Two triggers:
  stuck      for `timeout` seconds the module has neither moved more than
             `min_distance` nor turned more than `min_turn` (StuckDetector).
  off track  the module is more than `max_offset` sideways from the segment it
             is driving (last waypoint -> current target), e.g. after wheel slip
             (off_track()).
The controller then asks the Replanner for a new grid route from where the
module is now to its pre-dock point. The new route avoids
  - slots that are already assembled (as in navigation.plan_routes),
  - the grid nodes the other modules are on right now,
  - stuck only: the waypoint the module was stuck on (treated as blocked).
If no route avoids all of that, the other modules and the stuck waypoint are
dropped and it tries again. Each trigger has its own budget per module
(`max_replans`, `max_offtrack_replans`); when it is used up the Replanner gives
up (returns None).

This is not obstacle avoidance: nothing is sensed, a blockage is only inferred
from the module not making progress.
No ROS in this file.
"""
import math

from msrr_control.navigation import astar
from msrr_control.topology import wrap_angle


class StuckDetector:

    def __init__(self, timeout=5.0, min_distance=0.01, min_turn=0.1):
        self.timeout = timeout            # s without progress before a module is stuck
        self.min_distance = min_distance  # m of travel that counts as progress
        self.min_turn = min_turn          # rad of turning that counts as progress
        self.anchor = {}                  # module -> (pose, time) of the last progress

    def reset(self, module=None):
        """Forget a module's history (or every module's), e.g. after a replan or a pause."""
        if module is None:
            self.anchor.clear()
        else:
            self.anchor.pop(module, None)

    def update(self, module, pose, now):
        """Record the pose; return True if the module is stuck."""
        if module not in self.anchor:
            self.anchor[module] = (pose, now)
            return False
        (x0, y0, yaw0), t0 = self.anchor[module]
        moved = math.hypot(pose[0] - x0, pose[1] - y0)
        turned = abs(wrap_angle(pose[2] - yaw0))
        if moved > self.min_distance or turned > self.min_turn:
            self.anchor[module] = (pose, now)
            return False
        return now - t0 > self.timeout


def off_track(pose, seg_start, seg_end, max_offset):
    """Distance from pose to the segment seg_start -> seg_end if above max_offset, else None."""
    ax, ay = seg_start[0], seg_start[1]
    dx, dy = seg_end[0] - ax, seg_end[1] - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((pose[0] - ax) * dx
                                                      + (pose[1] - ay) * dy) / length2))
    offset = math.hypot(pose[0] - (ax + t * dx), pose[1] - (ay + t * dy))
    return offset if offset > max_offset else None


class Replanner:

    def __init__(self, grid, slots, max_replans=3, max_offtrack_replans=10):
        """grid: navigation.Grid. slots: slot -> pose in the root frame (topology.unfold)."""
        self.grid = grid
        self.limit = {'stuck': max_replans, 'off_track': max_offtrack_replans}
        self.slot_node = {s: (round(p[0] / grid.cell), round(p[1] / grid.cell))
                          for s, p in slots.items()}
        self.count = {}                   # (module, reason) -> replans so far

    def used(self, module, reason='stuck'):
        return self.count.get((module, reason), 0)

    def replan(self, module, pose, pre_dock, stuck_target, filled_slots, other_poses,
               reason='stuck'):
        """New world route [pose, grid nodes..., pre_dock], or None to give up.

        stuck_target: world (x, y) of the waypoint the module could not reach, or None
                      (off track: nothing is in the way, so no node is blocked for it).
        filled_slots: slots already assembled. other_poses: poses of every other module.
        reason: 'stuck' or 'off_track', each counted against its own budget.
        """
        n = self.used(module, reason)
        if n >= self.limit[reason]:
            return None
        self.count[(module, reason)] = n + 1

        start = self.grid.to_node(pose)
        goal = self.grid.to_node(pre_dock)
        blocked = {self.slot_node[s] for s in filled_slots}
        avoid = {self.grid.to_node(p) for p in other_poses}
        if stuck_target is not None:
            stuck_node = self.grid.to_node(stuck_target)
            if stuck_node not in (start, goal):
                avoid.add(stuck_node)

        for extra in (avoid, set()):
            try:
                nodes = astar(self.grid, start, goal, blocked | extra)
            except ValueError:
                continue
            return ([tuple(pose[:2])] + [self.grid.to_world(nd) for nd in nodes]
                    + [tuple(pre_dock[:2])])
        return None
