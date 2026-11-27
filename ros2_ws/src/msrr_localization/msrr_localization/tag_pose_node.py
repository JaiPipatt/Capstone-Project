"""
ROS 2 node: module poses from the overhead USB webcam AprilTags.

  ros2 launch msrr_localization localization.launch.py

For each tag in `tag_map` ('<tag id>:<module>') it publishes
  /<module>/<pose_topic>   nav_msgs/Odometry  pose in world_frame, twist in the body frame
  /<module>/<raw_topic>    geometry_msgs/PoseWithCovarianceStamped  unfiltered detection,
                           stamped with the frame arrival time (input of fusion_node)
  TF  world_frame -> <module>/base_link
and for RViz
  /tag/markers             visualization_msgs/MarkerArray  every tracked tag (grey = predicted)
  /tag/debug_image         sensor_msgs/Image   camera view with the tag_live_view overlay

Poses are Kalman filtered and extrapolated to the publish time (TagTracker). A tag
that is lost for `lost_after` s is no longer published.
Pixels -> metres: the homography from tag_calibrate (homography_file). Without
that file a plain scale m_per_px around the image centre is used.
"""
import math
import os
import threading

import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from msrr_localization.floor import FloorMap
from msrr_localization.tag_detection import wrap_angle
from msrr_localization.tag_live_view import draw
from msrr_localization.tag_tracker import TagTracker
from msrr_localization.usbcam import UsbCam

VEL_DT = 0.1   # s, finite difference used to map image velocities to the floor


def parse_tag_map(entries):
    # ['1:module_1', ...] -> {1: 'module_1', ...}
    out = {}
    for e in entries:
        tag_id, module = e.split(':')
        out[int(tag_id)] = module.strip()
    return out


class TagPoseNode(Node):

    def __init__(self):
        super().__init__('tag_pose_node')
        p = self.declare_parameter
        p('device', 'C270')                # webcam name (part of), /dev/videoN or index
        p('width', 1280)                   # must match the calibration
        p('height', 720)
        p('exposure', -1)                  # 100 us units; -1 = auto
        p('tag_map', ['1:module_1', '2:module_2', '3:module_3'])
        p('pose_topic', 'tag/odom')
        p('raw_topic', 'tag/pose_raw')
        p('world_frame', 'world')
        p('homography_file', '~/.ros/msrr_floor_homography.yaml')
        p('m_per_px', 0.001)               # fallback without homography_file
        p('heading_offset', 0.0)           # rad, module forward = tag front + offset
        p('lost_after', 0.5)               # s
        p('publish_image', True)
        p('image_scale', 0.5)
        p('image_every', 2)                # publish every n-th frame

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.device = g('device')
        self.width = g('width')
        self.height = g('height')
        self.exposure = g('exposure')
        self.tag_map = parse_tag_map(g('tag_map'))
        self.world_frame = g('world_frame')
        self.m_per_px = g('m_per_px')
        self.heading_offset = g('heading_offset')
        self.publish_image = g('publish_image')
        self.image_scale = g('image_scale')
        self.image_every = max(1, g('image_every'))

        self.floor = None
        path = os.path.expanduser(g('homography_file'))
        if path and os.path.exists(path):
            self.floor = FloorMap.load(path)
            self.get_logger().info(f'floor homography from {path}')
        else:
            self.get_logger().warn(f'no homography file {path}: using m_per_px={self.m_per_px} '
                                   'around the image centre (run tag_calibrate)')

        self.tracker = TagTracker(lost_after=g('lost_after'))
        self.odom_pubs = {
            module: self.create_publisher(Odometry, f'/{module}/{g("pose_topic")}', 10)
            for module in self.tag_map.values()}
        self.raw_pubs = {
            module: self.create_publisher(PoseWithCovarianceStamped,
                                          f'/{module}/{g("raw_topic")}', 10)
            for module in self.tag_map.values()}
        self.marker_pub = self.create_publisher(MarkerArray, '/tag/markers', 10)
        self.image_pub = self.create_publisher(Image, '/tag/debug_image', 2)
        self.tf = TransformBroadcaster(self)

        self.get_logger().info(
            f'camera {self.device} {self.width}x{self.height}, tags '
            + ', '.join(f'{i}->{m}' for i, m in self.tag_map.items()))
        self.thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ camera thread
    def camera_loop(self):
        cam = UsbCam(self.device, self.width, self.height)
        cam.configure(exposure=None if self.exposure < 0 else self.exposure)
        n = 0
        for frame, t_cam in cam.frames(log=self.get_logger().info):
            if not rclpy.ok():
                break
            t_frame = self.get_clock().now().to_msg()
            self.check_floor(frame)
            poses = self.tracker.process(frame, t_cam)
            self.publish_raw(t_frame)
            self.publish_poses(poses)
            n += 1
            if self.publish_image and n % self.image_every == 0:
                self.publish_debug_image(frame, poses)

    def check_floor(self, frame):
        size = (frame.shape[1], frame.shape[0])
        if self.floor is None:
            self.floor = FloorMap.from_scale(self.m_per_px, *size)
        elif self.floor.image_size is not None and self.floor.image_size != size:
            self.get_logger().error(
                f'image is {size[0]}x{size[1]} but the homography was fitted for '
                f'{self.floor.image_size[0]}x{self.floor.image_size[1]}: poses are wrong',
                throttle_duration_sec=5.0)

    # ------------------------------------------------------------ publishing
    def world_pose(self, p):
        """TagPose (px) -> world x, y, yaw, body-frame v_forward, v_left, omega."""
        x, y, yaw = self.floor.pose(p.x, p.y, p.theta)
        x2, y2, yaw2 = self.floor.pose(p.x + p.vx * VEL_DT, p.y + p.vy * VEL_DT,
                                       p.theta + p.omega * VEL_DT)
        vx, vy = (x2 - x) / VEL_DT, (y2 - y) / VEL_DT
        omega = wrap_angle(yaw2 - yaw) / VEL_DT
        yaw = wrap_angle(yaw + self.heading_offset)
        c, s = math.cos(yaw), math.sin(yaw)
        return x, y, yaw, c * vx + s * vy, -s * vx + c * vy, omega

    def publish_raw(self, stamp):
        """Unfiltered detections of this frame, for fusion_node."""
        for tag_id, d in self.tracker.detections.items():
            module = self.tag_map.get(tag_id)
            if module is None:
                continue
            x, y, yaw = self.floor.pose(d.x, d.y, d.theta)
            yaw = wrap_angle(yaw + self.heading_offset)
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.world_frame
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.pose.orientation.w = math.cos(yaw / 2)
            self.raw_pubs[module].publish(msg)

    def publish_poses(self, poses):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.world_frame
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for tag_id, p in sorted(poses.items()):
            x, y, yaw, v_fwd, v_left, omega = self.world_pose(p)
            qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
            module = self.tag_map.get(tag_id)
            markers.markers += self.tag_markers(stamp, tag_id, module, p, x, y, qz, qw)
            if module is None:
                continue

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.world_frame
            odom.child_frame_id = f'{module}/base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = v_fwd, v_left
            odom.twist.twist.angular.z = omega
            self.odom_pubs[module].publish(odom)

            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = odom.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            transforms.append(tf)

        if transforms:
            self.tf.sendTransform(transforms)
        self.marker_pub.publish(markers)

    def tag_markers(self, stamp, tag_id, module, p, x, y, qz, qw):
        color = (0.0, 1.0, 1.0) if p.detected else (0.6, 0.6, 0.6)
        if module is None:
            color = (1.0, 0.3, 0.3)   # tag not in tag_map
        out = []
        for ns, mtype in (('tag_arrows', Marker.ARROW), ('tag_labels', Marker.TEXT_VIEW_FACING)):
            mk = Marker()
            mk.header.stamp = stamp
            mk.header.frame_id = self.world_frame
            mk.ns, mk.id, mk.type, mk.action = ns, tag_id, mtype, Marker.ADD
            mk.pose.position.x, mk.pose.position.y = x, y
            mk.pose.orientation.z, mk.pose.orientation.w = qz, qw
            mk.color.r, mk.color.g, mk.color.b = color
            mk.color.a = 1.0
            if mtype == Marker.ARROW:
                mk.pose.position.z = 0.05
                mk.scale.x, mk.scale.y, mk.scale.z = 0.09, 0.015, 0.015
            else:
                mk.pose.position.z = 0.13
                mk.scale.z = 0.022
                mk.text = f'{module or "?"} tag {tag_id}' + ('' if p.detected else ' PRED')
            out.append(mk)
        return out

    def publish_debug_image(self, frame, poses):
        draw(frame, self.tracker, poses)
        if self.image_scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.image_scale, fy=self.image_scale)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tag_camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TagPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
