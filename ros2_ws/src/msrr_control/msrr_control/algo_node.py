"""
Central controller for N MSRR modules.

Reads each module's pose and sends it a velocity command, at a fixed rate.

Inputs  : /module_i/<pose_topic>                     nav_msgs/Odometry
          (sim: ground_truth/odom; later: AprilTag localization)
Outputs : /module_i/diff_drive_controller/cmd_vel_unstamped   geometry_msgs/Twist

Once every module has reported a pose, the self-assembly plan is computed
(root, slot assignment, rounds, grid routes). Rounds then run one after the
other; inside a round every moving module goes through
  navigate  follow its grid route to the pre-dock point
  adjust    turn in place to the goal heading
  approach  drive straight along its own x axis onto the goal
Moving modules do not avoid each other yet.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from msrr_control.assembly import assign_modules_to_slots, docking_tasks
from msrr_control.navigation import plan_routes
from msrr_control.topology import (
    check_topology, coordinates_to_frame, find_root, Topology, unfold, wrap_angle)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def parse_connection(text):
    # '1,B,0,T' -> (1, 'B', 0, 'T')
    u, cu, v, cv = (s.strip() for s in text.split(','))
    return int(u), cu, int(v), cv


class AlgoNode(Node):

    def __init__(self):
        super().__init__('algo_node')

        self.declare_parameter('num_modules', 3)
        self.declare_parameter('pose_topic', 'ground_truth/odom')
        self.declare_parameter('rate', 10.0)                # Hz
        self.declare_parameter('max_linear', 0.049)         # m/s, real top speed (50 RPM / 5.333)
        self.declare_parameter('max_angular', 1.0)          # rad/s
        self.declare_parameter('goal_tolerance', 0.01)      # m, pre-dock and dock positions
        self.declare_parameter('waypoint_tolerance', 0.02)  # m, intermediate grid nodes
        self.declare_parameter('heading_tolerance', 0.05)   # rad
        self.declare_parameter('module_width', 0.107)       # m, centre-to-centre when docked
        self.declare_parameter('grid', 0.107)               # m, navigation cell size
        # Target shape, one 'u,cu,v,cv' string per connection (slot u face cu -> slot v face cv)
        self.declare_parameter('connections', ['1,B,0,T', '2,B,0,L'])

        n = self.get_parameter('num_modules').value
        pose_topic = self.get_parameter('pose_topic').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.tolerance = self.get_parameter('goal_tolerance').value
        self.wp_tolerance = self.get_parameter('waypoint_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.width = self.get_parameter('module_width').value
        self.grid = self.get_parameter('grid').value
        connections = [parse_connection(c) for c in self.get_parameter('connections').value]

        self.modules = [f'module_{i}' for i in range(1, n + 1)]
        self.shape = Topology(n, connections)
        check_topology(self.shape, self.width)

        self.poses = {}     # module -> (x, y, yaw)
        self.rounds = None  # list of rounds, each a list of DockingTask
        self.routes = {}    # module -> world waypoints
        self.round_idx = 0
        self.state = {}     # module -> {'phase': ..., 'wp': index}

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

    def make_plan(self):
        modules = {m: self.poses[m] for m in self.modules}
        root, _ = find_root(self.shape)
        slots = unfold(self.shape, root, self.width)
        f, m_root = assign_modules_to_slots(slots, root, modules)
        self.rounds = docking_tasks(self.shape, root, f, slots, modules, m_root, self.grid)
        _, self.routes = plan_routes(self.rounds, f, root, slots, modules, m_root, self.grid)

        log = self.get_logger()
        log.info(f'root slot {root} -> {m_root}; mapping '
                 + ', '.join(f'{s}->{m}' for s, m in sorted(f.items())))
        for i, tasks in enumerate(self.rounds, 1):
            log.info(f'round {i}: ' + ', '.join(str(t.action) for t in tasks))
            for t in tasks:
                m, face = t.action[0], t.action[1]
                if face in ('L', 'R'):
                    # a differential drive cannot move sideways; the paper uses a helping module
                    log.warn(f'{m} would dock with its {face} face; skipping')
                    self.state[m] = {'phase': 'done', 'wp': 0}
                else:
                    self.state[m] = {'phase': 'navigate', 'wp': 0}

    def step(self):
        commands = {m: Twist() for m in self.modules}

        if self.rounds is None and len(self.poses) == len(self.modules):
            self.make_plan()

        if self.rounds is not None and self.round_idx < len(self.rounds):
            tasks = self.rounds[self.round_idx]
            for t in tasks:
                commands[t.action[0]] = self.task_command(t)
            if all(self.state[t.action[0]]['phase'] == 'done' for t in tasks):
                self.round_idx += 1
                self.get_logger().info(f'round {self.round_idx} done')
                if self.round_idx == len(self.rounds):
                    self.get_logger().info('assembly finished')

        for m, cmd in commands.items():
            self.cmd_pubs[m].publish(cmd)

    def task_command(self, task):
        m = task.action[0]
        st = self.state[m]
        pose = self.poses[m]
        cmd = Twist()

        if st['phase'] == 'navigate':
            route = self.routes[m]
            last = st['wp'] == len(route) - 1
            target = route[st['wp']]
            if math.hypot(target[0] - pose[0], target[1] - pose[1]) < (
                    self.tolerance if last else self.wp_tolerance):
                if last:
                    st['phase'] = 'adjust'
                else:
                    st['wp'] += 1
                return cmd
            return self.drive_to(pose, target)

        if st['phase'] == 'adjust':
            err = wrap_angle(task.goal[2] - pose[2])
            if abs(err) < self.heading_tolerance:
                st['phase'] = 'approach'
                return cmd
            cmd.angular.z = self.clamp(2.0 * err, self.max_angular)
            return cmd

        if st['phase'] == 'approach':
            # error in the goal's frame: drive along x, hold the heading
            ex, _, eth = coordinates_to_frame(task.goal, pose)
            if abs(ex) < self.tolerance:
                st['phase'] = 'done'
                self.get_logger().info(f'{m} docked')
                return cmd
            cmd.linear.x = self.clamp(-0.5 * ex, self.max_linear)  # negative: back in
            cmd.angular.z = self.clamp(-2.0 * eth, self.max_angular)
            return cmd

        return cmd  # done

    def drive_to(self, pose, target):
        # turn toward the point, drive forward once roughly facing it
        cmd = Twist()
        x, y, yaw = pose
        dist = math.hypot(target[0] - x, target[1] - y)
        heading_error = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)
        cmd.angular.z = self.clamp(2.0 * heading_error, self.max_angular)
        if abs(heading_error) < 0.3:
            cmd.linear.x = min(self.max_linear, 0.5 * dist)
        return cmd

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))


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
