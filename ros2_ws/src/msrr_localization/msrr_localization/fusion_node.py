"""
ROS 2 node: fuse each module's wheel encoders with the overhead camera (FusionFilter).

  ros2 run msrr_localization fusion_node --ros-args --params-file config/fusion.yaml

Per module in `modules`:
  in   /<module>/<encoder_topic>   geometry_msgs/Twist   measured v, w (ESP32, 20 Hz)
  in   /<module>/<camera_topic>    geometry_msgs/PoseWithCovarianceStamped
                                   raw tag pose at the frame time (tag_pose_node)
  out  /<module>/<output_topic>    nav_msgs/Odometry     fused pose + encoder twist
       TF world_frame -> <module>/base_link_fused   (publish_tf)

Without encoder messages the filter keeps working from the camera alone (motion is
then treated as unknown). Without camera it dead-reckons on the encoders for
`max_dead_reckoning` s, then stops publishing so algo_node's pose_timeout stops
the module.
"""
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from msrr_localization.fusion_filter import FusionFilter

FILTER_PARAMS = ('cam_pos_std', 'cam_yaw_std', 'enc_v_std', 'enc_w_std', 'enc_v_rel',
                 'enc_w_rel', 'encoder_timeout', 'stale_v_std', 'stale_w_std', 'gate_chi2',
                 'reset_after', 'horizon')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')
        p = self.declare_parameter
        p('modules', ['module_1', 'module_2', 'module_3'])
        p('encoder_topic', 'esp32/vel_state')
        p('camera_topic', 'tag/pose_raw')
        p('output_topic', 'fused/odom')
        p('world_frame', 'world')
        p('rate', 30.0)                    # Hz, output
        p('camera_delay', 0.03)            # s, exposure + USB before the frame reaches us
        p('encoder_delay', 0.02)           # s, STM32 -> ESP32 -> WiFi
        p('encoder_v_sign', 1.0)           # -1 if forward driving shows negative v
        p('encoder_w_sign', 1.0)           # -1 if turning left shows negative w
        p('max_dead_reckoning', 1.0)       # s without camera before output stops
        p('publish_tf', True)
        defaults = FusionFilter.__init__.__defaults__
        for name, value in zip(FILTER_PARAMS, defaults):
            p(name, value)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.world_frame = g('world_frame')
        self.camera_delay = g('camera_delay')
        self.encoder_delay = g('encoder_delay')
        self.v_sign, self.w_sign = g('encoder_v_sign'), g('encoder_w_sign')
        self.max_dr = g('max_dead_reckoning')
        self.publish_tf = g('publish_tf')
        kwargs = {name: g(name) for name in FILTER_PARAMS}

        self.filters, self.pubs = {}, {}
        for m in g('modules'):
            self.filters[m] = FusionFilter(**kwargs)
            self.pubs[m] = self.create_publisher(Odometry, f'/{m}/{g("output_topic")}', 10)
            self.create_subscription(Twist, f'/{m}/{g("encoder_topic")}',
                                     lambda msg, m=m: self.on_encoder(m, msg), 20)
            self.create_subscription(PoseWithCovarianceStamped, f'/{m}/{g("camera_topic")}',
                                     lambda msg, m=m: self.on_camera(m, msg), 20)
        self.tf = TransformBroadcaster(self)
        self.create_timer(1.0 / g('rate'), self.publish)
        self.get_logger().info(
            f'fusing /<module>/{g("encoder_topic")} + /<module>/{g("camera_topic")} '
            f'-> /<module>/{g("output_topic")} for {", ".join(self.filters)}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_encoder(self, m, msg):
        self.filters[m].add_encoder(self.now() - self.encoder_delay,
                                    self.v_sign * msg.linear.x, self.w_sign * msg.angular.z)

    def on_camera(self, m, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.camera_delay
        pose = msg.pose.pose
        self.filters[m].add_camera(t, pose.position.x, pose.position.y,
                                   yaw_of(pose.orientation))

    def publish(self):
        now = self.now()
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for m, f in self.filters.items():
            if not f.initialized or now - f.t_cam > self.max_dr:
                continue
            x, y, yaw, P = f.state_at(now)
            v, w = f.velocity(now)
            qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.world_frame
            odom.child_frame_id = f'{m}/base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
            cov = [0.0] * 36
            for i, j in ((0, 0), (0, 1), (1, 0), (1, 1)):
                cov[6 * i + j] = float(P[i, j])
            cov[0 * 6 + 5] = cov[5 * 6 + 0] = float(P[0, 2])
            cov[1 * 6 + 5] = cov[5 * 6 + 1] = float(P[1, 2])
            cov[5 * 6 + 5] = float(P[2, 2])
            odom.pose.covariance = cov
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
            self.pubs[m].publish(odom)

            if self.publish_tf:
                tf = TransformStamped()
                tf.header = odom.header
                tf.child_frame_id = f'{m}/base_link_fused'
                tf.transform.translation.x, tf.transform.translation.y = x, y
                tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
                transforms.append(tf)
        if transforms:
            self.tf.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
