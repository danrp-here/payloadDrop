#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

USAGE = """
Usage:
  ros2 run payload_ground_stack nav_to -- <lat_deg> <lon_deg> <alt_m>
Example:
  ros2 run payload_ground_stack nav_to -- -6.8930 107.6100 750
"""

class NavTo(Node):
    def __init__(self, lat, lon, alt):
        super().__init__('nav_to_cmd')
        self.pub = self.create_publisher(Point, '/cmd/target_lla', 10)
        p = Point(); p.x=float(lat); p.y=float(lon); p.z=float(alt)
        self.get_logger().info(f'Sending target LLA: lat={p.x:.6f}, lon={p.y:.6f}, alt={p.z:.1f} m')
        # publish a few times to ensure delivery
        def send_once():
            self.pub.publish(p)
        for _ in range(10):
            self.create_timer(0.05*(_+1), send_once)
        # stop after short delay
        self.create_timer(0.7, self.finish)

    def finish(self):
        self.get_logger().info('nav_to command sent.')
        rclpy.shutdown()

def main():
    if len(sys.argv) < 4+1:  # script + 3 args
        print(USAGE); return
    _, lat, lon, alt = sys.argv[:4+1]
    rclpy.init()
    node = NavTo(lat, lon, alt)
    rclpy.spin(node)
