#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Map2OdomPublisher(Node):
    def __init__(self):
        super().__init__('map2odom_publisher')
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            TransformStamped,
            '/radar_graph_slam/odom2pub',
            self.callback,
            10)
        self.odom_msg = None
        # timer to keep publishing at 10 Hz even without updates
        self.timer = self.create_timer(0.1, self.spin_once)

    def callback(self, msg):
        self.odom_msg = msg

    def spin_once(self):
        if self.odom_msg is None:
            ts = TransformStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.header.frame_id = 'map'
            ts.child_frame_id = 'odom'
            ts.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(ts)
            return

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.odom_msg.header.frame_id
        ts.child_frame_id = self.odom_msg.child_frame_id
        ts.transform = self.odom_msg.transform
        self.broadcaster.sendTransform(ts)

def main():
    rclpy.init()
    node = Map2OdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()