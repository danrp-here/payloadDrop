#!/usr/bin/env python3
import rclpy, sys, time
from rclpy.node import Node
from geometry_msgs.msg import Point

USAGE = """
Usage:
  ros2 run payload_ground_stack nav_here
Sets the target to the payload's current NED position (needs /state/pose_ned).
"""

class NavHere(Node):
    def __init__(self):
        super().__init__('nav_here_cmd')
        self.pub = self.create_publisher(Point, '/cmd/target_ned', 10)
        self.sub = self.create_subscription(Point, '/state/pose_ned', self.cb, 10)
        self.seen = None
        self.create_timer(3.0, self.timeout)

    def cb(self, msg: Point):
        self.seen = msg
        p = Point(); p.x=msg.x; p.y=msg.y; p.z=msg.z
        self.get_logger().info(f'Locking target to HERE: N={p.x:.1f} E={p.y:.1f} D={p.z:.1f}')
        for _ in range(5):
            self.pub.publish(p)
            time.sleep(0.05)
        rclpy.shutdown()

    def timeout(self):
        if self.seen is None:
            self.get_logger().error('No /state/pose_ned yet. Is the bridge running and GPS origin set?')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = NavHere()
    rclpy.spin(node)
