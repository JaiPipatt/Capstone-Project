"""
Central controller for N MSRR modules.

Reads each module's pose and sends it a velocity command, at a fixed rate.

Inputs  : /module_i/<pose_topic>                     nav_msgs/Odometry
          (sim: ground_truth/odom; real: tag/odom from msrr_localization)
          /module_i/joint_states                     sensor_msgs/JointState
Outputs : /module_i/diff_drive_controller/cmd_vel_unstamped   geometry_msgs/Twist
          /algo/markers                              visualization_msgs/MarkerArray
          (RViz: the plan, and every module as the controller sees it)

Once every module has reported a pose, the self-assembly plan is computed
(root, slot assignment, rounds, grid routes). Rounds then run one after the
other; inside a round every moving module goes through
  navigate  follow its grid route to the pre-dock point
  adjust    turn in place to the goal heading
  approach  drive straight along its own x axis onto the goal
For an L/R (wheel-face) docking, after navigate:
  wheel_dock  pivot -> straight -> pivot -> straight (wheel_dock_plan.py,
              wheel_dock.py)
A module whose pose is older than pose_timeout is stopped until it is seen again.
Moving modules do not avoid each other yet.
"""
import math

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from msrr_control.assembly import assign_modules_to_slots, docking_tasks
from msrr_control.navigation import plan_routes
from msrr_control.topology import (
    check_topology, coordinates_from_frame, coordinates_to_frame, find_root, Topology,
    unfold, wrap_angle)
from msrr_control.wheel_dock import WheelDocker

PHASE_COLOR = {
    'wait': (0.6, 0.6, 0.6),
    'navigate': (1.0, 0.6, 0.0),
    'adjust': (1.0, 0.6, 0.0),
    'approach': (1.0, 0.85, 0.0),
    'wheel_dock': (0.8, 0.3, 1.0),
    'done': (0.2, 0.8, 0.2),
    'failed': (1.0, 0.1, 0.1),
    'stale': (1.0, 0.1, 0.1),
}
ROUND_COLOR = [(0.2, 0.6, 1.0), (1.0, 0.5, 0.2), (0.4, 0.9, 0.4),
               (0.9, 0.3, 0.9), (0.9, 0.9, 0.2), (0.3, 0.9, 0.9)]


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
        self.declare_parameter('pose_timeout', 0.0)         # s, stop a module not seen; 0 = off
        self.declare_parameter('world_frame', 'world')      # frame of the poses, for RViz
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
        self.pose_timeout = self.get_parameter('pose_timeout').value
        self.world_frame = self.get_parameter('world_frame').value
        connections = [parse_connection(c) for c in self.get_parameter('connections').value]

        self.modules = [f'module_{i}' for i in range(1, n + 1)]
        self.shape = Topology(n, connections)
        check_topology(self.shape, self.width)

        self.poses = {}      # module -> (x, y, yaw)
        self.pose_time = {}  # module -> time the pose arrived (s, node clock)
        self.joints = {}     # module -> {joint name: (position, velocity)}
        self.rounds = None   # list of rounds, each a list of DockingTask
        self.routes = {}     # module -> world waypoints
        self.round_idx = 0
        self.state = {}      # module -> {'phase': ..., 'wp': index}
        self.nav_grid = None     # navigation.Grid, for drawing
        self.slot_world = {}     # slot -> world pose of the target slot
        self.slot_round = {}     # slot -> round index (-1 for the root)

        self.cmd_pubs = {}
        for m in self.modules:
            self.create_subscription(
                Odometry, f'/{m}/{pose_topic}', lambda msg, m=m: self.on_pose(m, msg), 10)
            self.create_subscription(
                JointState, f'/{m}/joint_states', lambda msg, m=m: self.on_joints(m, msg), 50)
            self.cmd_pubs[m] = self.create_publisher(
                Twist, f'/{m}/diff_drive_controller/cmd_vel_unstamped', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/algo/markers', 10)

        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(f'controlling {n} modules, pose from /module_i/{pose_topic}')

    def on_pose(self, module, msg):
        p = msg.pose.pose
        self.poses[module] = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.pose_time[module] = self.now()

    def on_joints(self, module, msg):
        vel = list(msg.velocity) + [0.0] * (len(msg.name) - len(msg.velocity))
        self.joints[module] = {n: (q, v) for n, q, v in zip(msg.name, msg.position, vel)}

    def make_plan(self):
        modules = {m: self.poses[m] for m in self.modules}
        root, _ = find_root(self.shape)
        slots = unfold(self.shape, root, self.width)
        f, m_root = assign_modules_to_slots(slots, root, modules)
        self.rounds = docking_tasks(self.shape, root, f, slots, modules, m_root, self.grid)
        # wheel-face docking pivots about a wheel, so it needs 2 cells of room, not 1
        for tasks in self.rounds:
            for t in tasks:
                if t.action[1] in ('L', 'R'):
                    gx, gy, _ = t.goal
                    t.pre_dock = (2 * t.pre_dock[0] - gx, 2 * t.pre_dock[1] - gy, t.pre_dock[2])
        self.nav_grid, self.routes = plan_routes(
            self.rounds, f, root, slots, modules, m_root, self.grid)

        self.slot_world = {s: coordinates_from_frame(modules[m_root], p) for s, p in slots.items()}
        slot_of = {m: s for s, m in f.items()}
        self.slot_round = {root: -1}
        for i, tasks in enumerate(self.rounds):
            for t in tasks:
                self.slot_round[slot_of[t.action[0]]] = i

        log = self.get_logger()
        log.info(f'root slot {root} -> {m_root}; mapping '
                 + ', '.join(f'{s}->{m}' for s, m in sorted(f.items())))
        for i, tasks in enumerate(self.rounds, 1):
            log.info(f'round {i}: ' + ', '.join(str(t.action) for t in tasks))
            for t in tasks:
                self.state[t.action[0]] = {'phase': 'navigate', 'wp': 0}

    def step(self):
        commands = {m: Twist() for m in self.modules}

        if self.rounds is None and len(self.poses) == len(self.modules):
            self.make_plan()

        stale = self.stale_modules()
        if stale:
            self.get_logger().warn(f'no recent pose for {sorted(stale)}: stopped',
                                   throttle_duration_sec=2.0)

        if self.rounds is not None and self.round_idx < len(self.rounds):
            tasks = self.rounds[self.round_idx]
            for t in tasks:
                if t.action[0] not in stale:
                    commands[t.action[0]] = self.task_command(t)
            if all(self.state[t.action[0]]['phase'] == 'done' for t in tasks):
                self.round_idx += 1
                self.get_logger().info(f'round {self.round_idx} done')
                if self.round_idx == len(self.rounds):
                    self.get_logger().info('assembly finished')

        for m, cmd in commands.items():
            self.cmd_pubs[m].publish(cmd)
        self.publish_markers(stale)

    def stale_modules(self):
        if self.pose_timeout <= 0.0:
            return set()
        now = self.now()
        return {m for m, t in self.pose_time.items() if now - t > self.pose_timeout}

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
                    if task.action[1] in ('L', 'R'):
                        return self.wheel_dock_start(task, st)
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

        if st['phase'] == 'wheel_dock':
            return self.wheel_dock_step(task, st)

        return cmd  # done or failed

    # ------------------------------------------------------------ wheel-face docking
    def wheel_angle(self, module, face):
        joint = 'left_wheel_joint' if face == 'L' else 'right_wheel_joint'
        return self.joints[module][joint][0]

    def wheel_dock_start(self, task, st):
        m, face, parent, _ = task.action
        if m not in self.joints or parent not in self.joints:
            self.get_logger().warn(f'{m}: waiting for joint_states of {m} and {parent} '
                                   'for wheel-face docking', throttle_duration_sec=5.0)
            return Twist()
        st['docker'] = WheelDocker(face, task.goal, self.max_linear)
        st['phase'] = 'wheel_dock'
        return Twist()

    def wheel_dock_step(self, task, st):
        m, face, parent, parent_face = task.action
        docker = st['docker']
        v, omega = docker.update(
            self.poses[m], self.wheel_angle(m, face),
            self.poses[parent], self.wheel_angle(parent, parent_face))
        if docker.failed:
            st['phase'] = 'failed'
            self.get_logger().error(f'{m} wheel dock failed: {docker.message}')
        elif docker.done:
            st['phase'] = 'done'
            self.get_logger().info(f'{m} {docker.message}')
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, omega
        return cmd

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

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

    # ------------------------------------------------------------ RViz
    def marker(self, ns, mid, mtype, color, alpha=1.0, pose=None, z=0.0):
        mk = Marker()
        mk.header.frame_id = self.world_frame
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns, mk.id, mk.type, mk.action = ns, mid, mtype, Marker.ADD
        mk.color.r, mk.color.g, mk.color.b = color
        mk.color.a = alpha
        x, y, yaw = pose if pose is not None else (0.0, 0.0, 0.0)
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(x), float(y), z
        mk.pose.orientation.z = math.sin(yaw / 2)
        mk.pose.orientation.w = math.cos(yaw / 2)
        return mk

    @staticmethod
    def points(xys, z=0.0):
        return [Point(x=float(p[0]), y=float(p[1]), z=z) for p in xys]

    def publish_markers(self, stale):
        out = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.world_frame
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        add = out.markers.append
        w = self.width

        # modules, at the pose the controller is using
        for i, m in enumerate(self.modules):
            if m not in self.poses:
                continue
            phase = 'stale' if m in stale else self.state.get(m, {}).get('phase', 'wait')
            color = PHASE_COLOR.get(phase, (0.6, 0.6, 0.6))
            body = self.marker('modules', i, Marker.CUBE, color, 0.8, self.poses[m], 0.015)
            body.scale.x, body.scale.y, body.scale.z = 0.95 * w, 0.95 * w, 0.03
            add(body)
            arrow = self.marker('heading', i, Marker.ARROW, (1.0, 1.0, 1.0), 1.0,
                                self.poses[m], 0.035)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.7 * w, 0.012, 0.012
            add(arrow)
            label = self.marker('labels', i, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0), 1.0,
                                self.poses[m], 0.09)
            label.scale.z = 0.025
            label.text = f'{m}\n{phase}'
            add(label)

        if self.rounds is None:
            self.marker_pub.publish(out)
            return

        # target slots, coloured by round (root: white)
        for s, pose in self.slot_world.items():
            r = self.slot_round.get(s, -1)
            color = (1.0, 1.0, 1.0) if r < 0 else ROUND_COLOR[r % len(ROUND_COLOR)]
            slot = self.marker('slots', s, Marker.CUBE, color, 0.25, pose, 0.002)
            slot.scale.x, slot.scale.y, slot.scale.z = w, w, 0.004
            add(slot)

        # grid roads
        g = self.nav_grid
        lines = self.marker('grid', 0, Marker.LINE_LIST, (0.7, 0.7, 0.7), 0.3)
        lines.scale.x = 0.001
        h = g.half_size
        for k in range(-h, h + 1):
            lines.points += self.points([g.to_world((k, -h)), g.to_world((k, h)),
                                         g.to_world((-h, k)), g.to_world((h, k))])
        add(lines)

        for r, tasks in enumerate(self.rounds):
            color = ROUND_COLOR[r % len(ROUND_COLOR)]
            current = r == self.round_idx
            for t in tasks:
                m = t.action[0]
                mid = self.modules.index(m)
                st = self.state[m]
                route = self.marker('routes', mid, Marker.LINE_STRIP, color,
                                    1.0 if current else 0.3, z=0.005)
                route.scale.x = 0.004
                route.points = self.points(self.routes[m])
                add(route)
                pre = self.marker('pre_dock', mid, Marker.SPHERE, color, 0.8, t.pre_dock, 0.005)
                pre.scale.x = pre.scale.y = pre.scale.z = 0.015
                add(pre)
                if current and st['phase'] == 'navigate':
                    wp = self.marker('waypoint', mid, Marker.SPHERE, (1.0, 1.0, 1.0), 1.0,
                                     (*self.routes[m][st['wp']], 0.0), 0.01)
                    wp.scale.x = wp.scale.y = wp.scale.z = 0.02
                    add(wp)
                docker = st.get('docker')
                if docker is not None and docker.plan is not None and st['phase'] == 'wheel_dock':
                    plan = docker.plan
                    path = self.marker('wheel_dock', mid, Marker.LINE_STRIP,
                                       PHASE_COLOR['wheel_dock'], 1.0, z=0.006)
                    path.scale.x = 0.003
                    path.points = self.points([plan.p0, plan.p1, plan.p_f])
                    add(path)
                    pts = self.marker('wheel_dock_points', mid, Marker.SPHERE_LIST,
                                      PHASE_COLOR['wheel_dock'], 1.0, z=0.006)
                    pts.scale.x = pts.scale.y = pts.scale.z = 0.012
                    pts.points = path.points
                    add(pts)

        self.marker_pub.publish(out)


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
