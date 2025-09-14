#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32, Float32MultiArray
from pymavlink import mavutil

R_EARTH = 6378137.0  # meters

class PayloadMavBridge(Node):
    def __init__(self):
        super().__init__('payload_mav_bridge')
        # Connection & IDs
        self.declare_parameter('conn_url', '/dev/ttyUSB1:57600')
        self.declare_parameter('payload_sysid', 72)
        # 2-servo params
        self.declare_parameter('servo_left', 1)
        self.declare_parameter('servo_right', 2)
        # 4-servo params
        self.declare_parameter('servo1', 1)
        self.declare_parameter('servo2', 2)
        self.declare_parameter('servo3', 3)
        self.declare_parameter('servo4', 4)
        # PWM map
        self.declare_parameter('pwm_center', 1500)
        self.declare_parameter('pwm_scale', 400)

        url = self.get_parameter('conn_url').value
        self.sysid = int(self.get_parameter('payload_sysid').value)
        self.servo_left  = int(self.get_parameter('servo_left').value)
        self.servo_right = int(self.get_parameter('servo_right').value)
        self.servo_idx4 = [int(self.get_parameter(n).value) for n in ('servo1','servo2','servo3','servo4')]
        self.center = int(self.get_parameter('pwm_center').value)
        self.scale  = int(self.get_parameter('pwm_scale').value)

        self.get_logger().info(f'Connecting MAVLink: {url}')
        self.mav = mavutil.mavlink_connection(url)
        self.mav.wait_heartbeat()
        self.get_logger().info('MAVLink heartbeat OK')

        # Publishers
        self.pub_pose  = self.create_publisher(Point, '/state/pose_ned', 10)
        self.pub_att   = self.create_publisher(Vector3, '/state/attitude', 10)
        self.pub_rates = self.create_publisher(Vector3, '/state/rates', 10)
        self.pub_vel   = self.create_publisher(Float32, '/state/vel', 10)

        # Subscribers (either 2-servo Vector3 or 4-servo Float32MultiArray)
        self.create_subscription(Vector3, '/cmd/surfaces_norm_vec', self.cb_two_servo, 10)
        self.create_subscription(Float32MultiArray, '/cmd/surfaces_norm_4', self.cb_four_servo, 10)

        # NED origin
        self.origin_set = False
        self.lat0 = 0.0; self.lon0 = 0.0; self.alt0 = 0.0

        # Timers
        self.create_timer(0.02, self.poll)  # 50 Hz
        self.create_timer(1.0,  self.send_heartbeat)

    # Heartbeat to be nice on the link
    def send_heartbeat(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE)

    def norm_to_pwm(self, x: float) -> int:
        x = max(-1.0, min(1.0, float(x)))
        return int(self.center + self.scale * x)

    # 2-servo Vector3: x=left, y=right
    def cb_two_servo(self, msg: Vector3):
        try:
            l = self.norm_to_pwm(msg.x); r = self.norm_to_pwm(msg.y)
            self.do_set_servo(self.servo_left, l)
            self.do_set_servo(self.servo_right, r)
        except Exception as e:
            self.get_logger().error(f'2-servo send err: {e}')

    # 4-servo Float32MultiArray: [s1,s2,s3,s4]
    def cb_four_servo(self, msg: Float32MultiArray):
        try:
            vals = list(msg.data)[:4]
            for idx, val in zip(self.servo_idx4, vals):
                self.do_set_servo(idx, self.norm_to_pwm(val))
        except Exception as e:
            self.get_logger().error(f'4-servo send err: {e}')

    def do_set_servo(self, servo_index: int, pwm_us: int):
        self.mav.mav.command_long_send(
            self.sysid,
            mavutil.mavlink.MAV_COMP_ID_PAYLOAD_CONTROLLER,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            float(servo_index), float(pwm_us), 0,0,0,0,0
        )

    # Simple LLH→NED
    def llh_to_ned(self, lat_deg, lon_deg, alt_m):
        if not self.origin_set:
            self.lat0, self.lon0, self.alt0 = lat_deg, lon_deg, alt_m
            self.origin_set = True
            return 0.0, 0.0, 0.0
        dlat = math.radians(lat_deg - self.lat0)
        dlon = math.radians(lon_deg - self.lon0)
        lat0 = math.radians(self.lat0)
        north = dlat * R_EARTH
        east  = dlon * R_EARTH * math.cos(lat0)
        down  = (alt_m - self.alt0)
        return north, east, down

    # MAVLink polling
    def poll(self):
        for _ in range(20):
            msg = self.mav.recv_match(blocking=False)
            if not msg: break
            t = msg.get_type()
            if t == 'ATTITUDE' and msg.get_srcSystem() == self.sysid:
                att = Vector3(); att.x = msg.roll; att.y = msg.pitch; att.z = msg.yaw
                rates = Vector3(); rates.x = msg.rollspeed; rates.y = msg.pitchspeed; rates.z = msg.yawspeed
                self.pub_att.publish(att); self.pub_rates.publish(rates)
            elif t == 'GPS_RAW_INT' and msg.get_srcSystem() == self.sysid:
                lat = msg.lat / 1e7; lon = msg.lon / 1e7; alt = msg.alt / 1000.0
                n,e,d = self.llh_to_ned(lat, lon, alt)
                pose = Point(); pose.x = float(n); pose.y = float(e); pose.z = float(d)
                self.pub_pose.publish(pose)
                v = Float32(); v.data = 0.0
                self.pub_vel.publish(v)

def main():
    rclpy.init()
    node = PayloadMavBridge()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()
