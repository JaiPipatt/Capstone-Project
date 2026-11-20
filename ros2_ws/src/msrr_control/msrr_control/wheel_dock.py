"""
Runs the wheel-face docking plan (see wheel_dock_plan.py) one control tick at a time.

Call update() every tick with the measured pose and wheel angle. It returns
the (v, omega) to command. When it is finished, `done` or `failed` is set.

Not done yet: EKF (the pose given is used as is), asking the parent to rotate
its face when the wheel angle is off, EP-face activation and retries.
"""
import math

from msrr_control import wheel_dock_plan as wp

# ------------------------------------------------------------------ tolerances and gains
TOL_POS = 0.003      # m, end of a straight
TOL_YAW = 0.03       # rad, end of a pivot
TOL_Q = 0.2       # rad, final wheel angle error (mod pi/2) that is still fine (~10 mm of rolling)
REPLAN_TOL = 0.01    # m, replan if a segment ends this far from the plan
K_YAW = 2.0
K_LATERAL = 5.0      # steer back onto the line
K_DIST = 2.0
MIN_SPEED = 0.01     # m/s, so the module does not stall near the end of a segment
MIN_TURN = 0.1       # rad/s


def clamp(value, limit):
    return max(-limit, min(limit, value))


class WheelDocker:

    def __init__(self, d, slot_pose, max_speed):
        """d: 'L' or 'R' docking wheel. slot_pose: where the body should end up."""
        self.d = d
        self.slot_pose = slot_pose
        self.max_speed = max_speed        # m/s of one wheel
        self.plan = None
        self.segments = []                # [(kind, ...)], kind is 'pivot' or 'straight'
        self.done = False
        self.failed = False
        self.message = ''
        self.q_error = 0.0

    # ------------------------------------------------------------ each tick
    def update(self, pose, q, parent_pose, parent_face_q):
        """Return (v, omega)."""
        if self.done or self.failed:
            return 0.0, 0.0

        if self.plan is None:
            goal = wp.goal_from_partner(self.slot_pose, parent_face_q, self.d)
            self.plan = wp.plan(pose, q, self.d, goal, parent_pose)
            if self.plan is None:
                self.failed, self.message = True, 'no clear docking plan'
                return 0.0, 0.0
            self.segments = self.make_segments(self.plan)
            self.goal = goal

        segment = self.segments[0]
        if segment.is_finished(pose, self.d):
            self.segments.pop(0)
            if not self.segments:
                return self.finish(pose, q)
            if self.off_plan(pose, segment):
                self.plan = None          # replan from where we really are
            return 0.0, 0.0

        return segment.command(pose, self.d, self.max_speed)

    def finish(self, pose, q):
        self.q_error = wp.wrap_step(q - self.goal.q_f)
        self.done = True
        # a bad wheel angle would need the parent to rotate its face (not done yet)
        self.message = 'docked' if abs(self.q_error) < TOL_Q else (
            f'docked but wheel angle off by {self.q_error:+.2f} rad')
        return 0.0, 0.0

    def off_plan(self, pose, segment):
        wheel = wp.wheel_centre(pose, self.d)
        return float(((wheel - segment.end) ** 2).sum() ** 0.5) > REPLAN_TOL

    @staticmethod
    def make_segments(plan):
        return [
            Pivot(plan.yaw1, plan.p0),
            Straight(plan.p0, plan.yaw1, plan.dist1, plan.p1),
            Pivot(plan.yaw_f, plan.p1),
            Straight(plan.p1, plan.yaw_f, plan.dist2, plan.p_f),
        ]


# ------------------------------------------------------------------ segments
class Pivot:
    """Hold wheel d still and turn the body to target_yaw."""

    def __init__(self, target_yaw, wheel_pos):
        self.target_yaw = target_yaw
        self.end = wheel_pos

    def error(self, pose):
        return wp.wrap(self.target_yaw - pose[2])

    def is_finished(self, pose, d):
        return abs(self.error(pose)) < TOL_YAW

    def command(self, pose, d, max_speed):
        error = self.error(pose)
        omega_max = max_speed / wp.WHEEL_SEPARATION      # the moving wheel is the limit
        omega = math.copysign(clamp(max(abs(K_YAW * error), MIN_TURN), omega_max), error)
        # the held wheel has speed 0:  left: v - omega*W/2 = 0,  right: v + omega*W/2 = 0
        v = omega * wp.WHEEL_SEPARATION / 2 * (1 if d == 'L' else -1)
        return v, omega


class Straight:
    """Drive wheel d along the line from `start` by a signed distance (negative = reverse)."""

    def __init__(self, start, yaw, distance, end):
        self.start = start
        self.yaw = yaw
        self.distance = distance
        self.end = end

    def progress(self, pose, d):
        offset = wp.wheel_centre(pose, d) - self.start
        along = offset @ wp.forward(self.yaw)
        off_line = offset @ wp.left(self.yaw)
        return self.distance - along, off_line      # distance left, sideways error

    def is_finished(self, pose, d):
        remaining, _ = self.progress(pose, d)
        return abs(remaining) < TOL_POS

    def command(self, pose, d, max_speed):
        remaining, off_line = self.progress(pose, d)
        direction = math.copysign(1.0, remaining)
        v = direction * clamp(max(K_DIST * abs(remaining), MIN_SPEED), max_speed)
        omega = K_YAW * wp.wrap(self.yaw - pose[2]) - direction * K_LATERAL * off_line
        return v, clamp(omega, 0.3)
