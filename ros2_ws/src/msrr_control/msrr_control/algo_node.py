"""
Central controller for N MSRR modules.

Reads each module's pose and sends it a velocity command, at a fixed rate.

Inputs  : /module_i/<pose_topic>                     nav_msgs/Odometry
          (sim: ground_truth/odom; later: AprilTag localization)
          /module_i/joint_states                     sensor_msgs/JointState
Outputs : /module_i/diff_drive_controller/cmd_vel_unstamped   geometry_msgs/Twist

Once every module has reported a pose, the self-assembly plan is computed
(root, slot assignment, rounds, grid routes). Rounds then run one after the
other; inside a round every moving module goes through
  navigate  follow its grid route to the pre-dock point
  adjust    turn in place to the goal heading
  approach  drive straight along its own x axis onto the goal
For an L/R (wheel-face) docking, after navigate (pre-dock point = p1 on l_f):
  wd_exec   plan pivot -> straight -> pivot -> straight (Wolfe et al. 2010,
            wheel_dock.py) from the measured pose and wheel angles and run it
            closed-loop (wheel_dock_exec.py); J is logged to results/wheel_dock.csv
Moving modules do not avoid each other yet.
"""
import csv
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from msrr_control.assembly import assign_modules_to_slots, docking_tasks
from msrr_control.navigation import plan_routes
from msrr_control.topology import (
    check_topology, coordinates_to_frame, find_root, Topology, unfold, wrap_angle)
from msrr_control import wheel_dock as wd
from msrr_control.wheel_dock_exec import ExecConfig, WheelDockExecutor

WD_PARAMS = {  # wheel_dock.* parameters and defaults, see config/algo.yaml
    'T': 30.0, 'N': 10, 'max_iter': 30, 'step_norm': 2.0, 'face_gap': 0.003,
    'min_final_straight': 0.16, 'pre_dock_s': 0.16, 'phi_f': 0.0,
    'axle_offset': [-0.004905, 0.0], 'wheel_radius': 0.05, 'wheel_separation': 0.09665,
    'rolling_radius': 0.0492, 'pivot_separation': 0.1028, 'results_dir': 'results',
}


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
        for k, v in WD_PARAMS.items():
            self.declare_parameter(f'wheel_dock.{k}', v)

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
        self.wdp = {k: self.get_parameter(f'wheel_dock.{k}').value for k in WD_PARAMS}
        r, w = self.wdp['wheel_radius'], self.wdp['wheel_separation']
        # self.wheel: diff_drive_controller's values, to turn wheel rates into cmd_vel
        # self.model: planner geometry (Gazebo-measured rolling radius and the
        # separation at which a held wheel actually pivots)
        self.wheel = wd.WheelParams(r=r, W=w, omega_max=self.max_linear / r)
        self.model = wd.WheelParams(r=self.wdp['rolling_radius'], W=self.wdp['pivot_separation'],
                                    omega_max=self.max_linear / r)
        self.run_id = time.strftime('%Y%m%d-%H%M%S')

        self.modules = [f'module_{i}' for i in range(1, n + 1)]
        self.shape = Topology(n, connections)
        check_topology(self.shape, self.width)

        self.poses = {}     # module -> (x, y, yaw)
        self.joints = {}    # module -> {joint name: (position, velocity)}
        self.rounds = None  # list of rounds, each a list of DockingTask
        self.routes = {}    # module -> world waypoints
        self.round_idx = 0
        self.state = {}     # module -> {'phase': ..., 'wp': index}

        self.cmd_pubs = {}
        for m in self.modules:
            self.create_subscription(
                Odometry, f'/{m}/{pose_topic}', lambda msg, m=m: self.on_pose(m, msg), 10)
            self.create_subscription(
                JointState, f'/{m}/joint_states', lambda msg, m=m: self.on_joints(m, msg), 50)
            self.cmd_pubs[m] = self.create_publisher(
                Twist, f'/{m}/diff_drive_controller/cmd_vel_unstamped', 10)

        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(f'controlling {n} modules, pose from /module_i/{pose_topic}')

    def on_pose(self, module, msg):
        p = msg.pose.pose
        self.poses[module] = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_joints(self, module, msg):
        vel = list(msg.velocity) + [0.0] * (len(msg.name) - len(msg.velocity))
        self.joints[module] = {n: (q, v) for n, q, v in zip(msg.name, msg.position, vel)}
        st = self.state.get(module)
        if st and st['phase'] == 'wd_exec':
            # measured wheel rates for J_executed (eq. 22)
            t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            j = self.joints[module]
            st['wd']['rates'].append(
                (t, -j['right_wheel_joint'][1], j['left_wheel_joint'][1]))

    def phi(self, module):
        j = self.joints.get(module, {})
        if 'left_wheel_joint' not in j:
            return None
        return wd.phi_from_joints(j['left_wheel_joint'][0], j['right_wheel_joint'][0])

    def axle_pose(self, module):
        return wd.axle_pose_from_base(self.poses[module], self.wdp['axle_offset'])

    def make_plan(self):
        modules = {m: self.poses[m] for m in self.modules}
        root, _ = find_root(self.shape)
        slots = unfold(self.shape, root, self.width)
        f, m_root = assign_modules_to_slots(slots, root, modules)
        self.rounds = docking_tasks(self.shape, root, f, slots, modules, m_root, self.grid)
        # wheel-face tasks: pre-dock point = p1 on l_f (end of l_f nearer the module)
        for tasks in self.rounds:
            for t in tasks:
                if t.action[1] in ('L', 'R'):
                    d, pf, th_f, _ = wd.dock_targets(t.goal, t.action[1], self.model,
                                                     self.wdp['axle_offset'], self.wdp['face_gap'])
                    start = modules[t.action[0]]
                    t.pre_dock = min(
                        (wd.pre_dock_base_pose(pf, th_f, d, s, self.model, self.wdp['axle_offset'])
                         for s in (self.wdp['pre_dock_s'], -self.wdp['pre_dock_s'])),
                        key=lambda q: math.hypot(q[0] - start[0], q[1] - start[1]))
        _, self.routes = plan_routes(self.rounds, f, root, slots, modules, m_root, self.grid)

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

        if st['phase'] == 'wd_exec':
            return self.wheel_dock_step(task, st)

        return cmd  # done

    # ------------------------------------------------------------ wheel-face docking
    def wheel_dock_start(self, task, st):
        m, face = task.action[0], task.action[1]
        phi0 = self.phi(m)
        if phi0 is None:
            return Twist()  # wait for joint_states
        p = self.model
        d, pf, th_f, pf_slot = wd.dock_targets(task.goal, face, p, self.wdp['axle_offset'],
                                               self.wdp['face_gap'])
        phi_f = self.wdp['phi_f']
        plan = wd.plan_nominal(self.axle_pose(m), phi0, d, pf, th_f, phi_f, p, self.wdp['T'],
                               min_final=self.wdp['min_final_straight'])      # Section II-C
        opt = wd.optimize(plan, N=self.wdp['N'], max_iter=self.wdp['max_iter'],
                          step_norm=self.wdp['step_norm'])                     # Section II-D
        st['wd'] = {'plan': plan, 'opt': opt, 'exec': WheelDockExecutor(plan, ExecConfig()),
                    'pf_slot': pf_slot, 't0': self.now(), 'rates': []}
        st['phase'] = 'wd_exec'
        log = self.get_logger()
        log.info(
            f'{m} wheel-dock face {face} (wheel {d}): k={plan.k} s={plan.s:+.3f} '
            f'dir01={plan.dir01:+d} T={plan.T:.1f}s J_nom={plan.J:.3f} J_opt={opt.J:.3f}')
        log.info(f'  {m} p0=({plan.p0[0]:+.3f}, {plan.p0[1]:+.3f})  '
                 f'p1=({plan.p1[0]:+.3f}, {plan.p1[1]:+.3f})  pf=({pf[0]:+.3f}, {pf[1]:+.3f})  '
                 f'theta_f={math.degrees(th_f):+.1f} deg')
        log.info(f'  {m} phi_d start {phi0[d - 1]:+.3f} rad -> target {phi_f:+.3f} '
                 f'(mod pi/2); omega_s={plan.omega_s:.3f} omega_p={plan.omega_p:.3f} rad/s')
        log.info(f'  {m} optimisation: {opt.iterations} iterations, J {opt.J_nominal:.3f} -> '
                 f'{opt.J:.3f} ({100 * (1 - opt.J / opt.J_nominal):.0f}% lower)')
        for i, sg in enumerate(plan.segments, 1):
            log.info(f'  {m} segment {i}/{len(plan.segments)}: {self.describe(sg)}')
        w = st['wd']
        w['seg'] = 0
        w['seg_t0'] = self.now()
        return Twist()

    @staticmethod
    def describe(sg):
        if sg.kind == 'pivot':
            return (f'pivot   {math.degrees(sg.dtheta):+7.1f} deg  rates '
                    f'({sg.phidot[0]:+.2f}, {sg.phidot[1]:+.2f}) rad/s  {sg.duration:5.1f} s')
        return (f'straight {sg.direction * sg.distance:+.3f} m  dphi_d {sg.dphi_d:+.2f} rad  '
                f'rates ({sg.phidot[0]:+.2f}, {sg.phidot[1]:+.2f}) rad/s  {sg.duration:5.1f} s')

    def log_segment_done(self, m, w, g, phi):
        """Log how the segment that just finished ended versus its plan."""
        sg = w['plan'].segments[w['seg']]
        d = w['plan'].d
        pw = wd.wheel_centre(g, d, self.model)
        planned = wd.wheel_centre(sg.end, d, self.model)
        off = 1000 * np.linalg.norm(pw - planned)
        th_err = math.degrees(wrap_angle(g[2] - sg.end[2]))
        self.get_logger().info(
            f'  {m} segment {w["seg"] + 1} {sg.kind} done in {self.now() - w["seg_t0"]:.1f} s '
            f'(planned {sg.duration:.1f}): wheel-d off plan {off:.1f} mm, '
            f'theta err {th_err:+.2f} deg, phi_d {phi[d - 1]:+.3f} rad')
        w['seg'] += 1
        w['seg_t0'] = self.now()

    def wheel_dock_step(self, task, st):
        w = st['wd']
        m = task.action[0]
        g, phi = self.axle_pose(m), self.phi(m)
        rates = w['exec'].update(g, phi)
        while w['seg'] < min(w['exec'].idx, len(w['plan'].segments)):
            self.log_segment_done(m, w, g, phi)
        if w['exec'].done:
            self.wheel_dock_finish(task, st)
            return Twist()
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = wd.twist_from_phidot(rates[0], rates[1], self.wheel)
        return cmd

    def wheel_dock_finish(self, task, st):
        m, face = task.action[0], task.action[1]
        w = st['wd']
        plan = w['plan']
        g, phi = self.axle_pose(m), self.phi(m)
        err = wd.pose_error(g, plan.gf)
        e_phi = wd.wrap_mod(phi[plan.d - 1] - plan.phi_f, plan.modulus)
        rel = wd.wheel_centre(g, plan.d, self.model) - w['pf_slot']
        normal = float(-rel @ wd.face_normal(plan.gf[2], plan.d))  # includes face_gap
        parallel = float(rel @ wd.heading(plan.gf[2]))
        r = np.array(w['rates'])
        J_exec = float(0.5 * np.trapz(r[:, 1] ** 2 + r[:, 2] ** 2, r[:, 0])) if len(r) > 1 else 0.0
        row = {'run_id': self.run_id, 'module': m, 'face': face, 'd': plan.d, 'k': plan.k,
               's': plan.s, 'dir01': plan.dir01, 'T': plan.T, 'J_plan_nominal': plan.J,
               'J_plan_opt': w['opt'].J, 'J_executed': J_exec, 'err_x': err[0],
               'err_y': err[1], 'err_theta': err[2], 'err_phi_mod': e_phi,
               'err_normal': normal, 'err_parallel': parallel,
               'duration': self.now() - w['t0']}
        out = os.path.abspath(self.wdp['results_dir'])
        os.makedirs(out, exist_ok=True)
        path = os.path.join(out, 'wheel_dock.csv')
        new = not os.path.exists(path)
        with open(path, 'a', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=list(row))
            if new:
                writer.writeheader()
            writer.writerow(row)
        st['phase'] = 'done'
        self.get_logger().info(
            f'{m} docked (wheel face {face}): pos err {1000 * math.hypot(err[0], err[1]):.1f} mm '
            f'(normal {1000 * normal:.1f}, parallel {1000 * parallel:+.1f}), theta err '
            f'{math.degrees(err[2]):+.2f} deg, phi err {e_phi:+.4f} rad, '
            f'J nom/opt/exec {plan.J:.3f}/{w["opt"].J:.3f}/{J_exec:.3f}')

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
