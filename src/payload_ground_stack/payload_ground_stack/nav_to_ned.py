#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

USAGE = """
Usage:
  ros2 run payload_ground_stack nav_to_ned -- <north_m> <east_m> <down_m>
Example:
  ros2 run payload_ground_stack nav_to_ned -- 0 300 0
"""

class NavToNED(Node):
    def __init__(self, n, e, d):
        super().__init__('nav_to_ned_cmd')
        self.pub = self.create_publisher(Point, '/cmd/target_ned', 10)
        p = Point(); p.x=float(n); p.y=float(e); p.z=float(d)
        self.get_logger().info(f'Sending target NED: N={p.x:.1f} E={p.y:.1f} D={p.z:.1f} m')
        def send_once():
            self.pub.publish(p)
        for _ in range(10):
            self.create_timer(0.05*(_+1), send_once)
        self.create_timer(0.7, self.finish)

    def finish(self):
        self.get_logger().info('nav_to_ned command sent.')
        rclpy.shutdown()

def main():
    if len(sys.argv) < 3+1:
        print(USAGE); return
    _, n, e, d = sys.argv[:3+1]
    rclpy.init()
    node = NavToNED(n, e, d)
    rclpy.spin(node)
