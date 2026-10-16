"""
Central controller for N MSRR modules.

Reads each module's pose and sends it a velocity command, at a fixed rate.

Inputs  : /module_i/<pose_topic>                     nav_msgs/Odometry
          (sim: ground_truth/odom; later: AprilTag localization)
Outputs : /module_i/diff_drive_controller/cmd_vel_unstamped   geometry_msgs/Twist

The example behaviour drives every module to its goal with a simple
turn-then-drive controller. Replace `compute_command` with your algorithm.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class AlgoNode(Node):

    def __init__(self):
        super().__init__('algo_node')

        self.declare_parameter('num_modules', 3)
        self.declare_parameter('pose_topic', 'ground_truth/odom')
        self.declare_parameter('rate', 10.0)              # Hz
        self.declare_parameter('max_linear', 0.049)       # m/s, real top speed (50 RPM / 5.333)
        self.declare_parameter('max_angular', 1.0)        # rad/s
        self.declare_parameter('goal_tolerance', 0.01)    # m
        # Goals as a flat list [x1, y1, x2, y2, ...] in the world frame
        self.declare_parameter('goals', [0.0])

        n = self.get_parameter('num_modules').value
        pose_topic = self.get_parameter('pose_topic').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.tolerance = self.get_parameter('goal_tolerance').value

        flat = list(self.get_parameter('goals').value)
        self.modules = [f'module_{i}' for i in range(1, n + 1)]
        self.goals = {}
        if len(flat) == 2 * n:
            self.goals = {m: (flat[2 * k], flat[2 * k + 1]) for k, m in enumerate(self.modules)}
        else:
            self.get_logger().warn(
                f'goals needs {2 * n} numbers, got {len(flat)}; modules will stay still')

        self.poses = {}   # module -> (x, y, yaw)
        self.cmd_pubs = {}
        for m in self.modules:
            self.create_subscription(
                Odometry, f'/{m}/{pose_topic}', lambda msg, m=m: self.on_pose(m, msg), 10)
            self.cmd_pubs[m] = self.create_publisher(
                Twist, f'/{m}/diff_drive_controller/cmd_vel_unstamped', 10)

        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(f'controlling {n} modules, pose from /module_i/{pose_topic}')

    def on_pose(self, module, msg):
        p = msg.pose.pose
        self.poses[module] = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def step(self):
        for m in self.modules:
            if m not in self.poses:
                continue  # no pose yet
            self.cmd_pubs[m].publish(self.compute_command(m, self.poses[m]))

    def compute_command(self, module, pose):
        """Return the Twist for one module. Replace this with your algorithm."""
        cmd = Twist()
        if module not in self.goals:
            return cmd

        x, y, yaw = pose
        gx, gy = self.goals[module]
        dist = math.hypot(gx - x, gy - y)
        if dist < self.tolerance:
            return cmd  # arrived

        heading_error = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, 2.0 * heading_error))
        # Drive forward only when roughly facing the goal
        if abs(heading_error) < 0.3:
            cmd.linear.x = min(self.max_linear, 0.5 * dist)
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = AlgoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop all modules on exit
        for pub in node.cmd_pubs.values():
            pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
