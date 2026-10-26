#!/usr/bin/env python3
"""
Bridge: ROS 2 cmd_vel -> ESP32 over WiFi UDP, one ESP32 per module.

Inputs  : /module_i/diff_drive_controller/cmd_vel_unstamped   geometry_msgs/Twist
Outputs : UDP datagram "v,w\\n" (m/s, rad/s) to hosts[i-1]:port
          The ESP32 does the diff-drive wheel-speed maths.
"""
import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Esp32Bridge(Node):

    def __init__(self):
        super().__init__('esp32Bridge')

        self.declare_parameter('num_modules', 1)
        # One IP per module, module_1 first. Empty string = module not connected.
        self.declare_parameter('hosts', ['192.168.4.101'])
        self.declare_parameter('port', 4210)
        self.declare_parameter('send_rate', 20.0)    # Hz, resend latest command
        self.declare_parameter('cmd_timeout', 0.5)   # s, send zero if no new command

        n = self.get_parameter('num_modules').value
        hosts = list(self.get_parameter('hosts').value)
        port = self.get_parameter('port').value
        self.timeout = self.get_parameter('cmd_timeout').value

        if len(hosts) != n:
            self.get_logger().warn(f'hosts needs {n} entries, got {len(hosts)}')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = {}
        self.last_cmd = {}   # module -> (v, w, time)
        for i, host in enumerate(hosts[:n], start=1):
            m = f'module_{i}'
            if not host:
                continue
            self.addr[m] = (host, port)
            self.create_subscription(
                Twist, f'/{m}/diff_drive_controller/cmd_vel_unstamped',
                lambda msg, m=m: self.on_cmd(m, msg), 10)

        self.create_timer(1.0 / self.get_parameter('send_rate').value, self.send)
        self.get_logger().info(f'bridging {len(self.addr)} modules to ESP32, UDP port {port}')

    def on_cmd(self, module, msg):
        self.last_cmd[module] = (msg.linear.x, msg.angular.z, self.get_clock().now())

    def send(self):
        now = self.get_clock().now()
        for m, addr in self.addr.items():
            v, w = 0.0, 0.0
            if m in self.last_cmd:
                cv, cw, t = self.last_cmd[m]
                if (now - t).nanoseconds * 1e-9 <= self.timeout:
                    v, w = cv, cw
            try:
                self.sock.sendto(f'{v:.4f},{w:.4f}\n'.encode(), addr)
            except OSError as e:
                self.get_logger().warn(f'{m}: send failed: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
