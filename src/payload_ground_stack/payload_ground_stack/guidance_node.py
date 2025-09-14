import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32

def wrap_angle(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance_node')
        # Params
        self.declare_parameter('target_n', 0.0)
        self.declare_parameter('target_e', 0.0)
        self.declare_parameter('target_d', 0.0)
        self.declare_parameter('k_nav', 0.9)
        self.declare_parameter('theta0_deg', -2.0)
        self.declare_parameter('kz', 0.003)
        self.declare_parameter('kv', 0.02)
        self.declare_parameter('V_ref', 15.0)
        self.declare_parameter('phi_max_deg', 35.0)
        self.declare_parameter('theta_min_deg', -10.0)
        self.declare_parameter('theta_max_deg', 10.0)

        # Initialize target from params
        self.target_n = float(self.get_parameter('target_n').value)
        self.target_e = float(self.get_parameter('target_e').value)
        self.target_d = float(self.get_parameter('target_d').value)

        # State
        self.N = 0.0; self.E = 0.0; self.D = 0.0
        self.psi = 0.0  # yaw
        self.V = 10.0   # m/s default

        # Subs
        self.create_subscription(Point, '/state/pose_ned', self.cb_pose, 10)
        self.create_subscription(Vector3, '/state/attitude', self.cb_att, 10)
        self.create_subscription(Float32, '/state/vel', self.cb_vel, 10)
        self.create_subscription(Point, '/cmd/target_ned', self.cb_target, 10)  # NEW

        # Pubs
        self.sp_pub = self.create_publisher(Vector3, '/cmd/attitude_sp', 10)
        self.tgt_pub = self.create_publisher(Point, '/state/target_ned', 10)    # echo current target

        self.timer = self.create_timer(0.02, self.step)  # 50 Hz
        self.get_logger().info('guidance_node started')

    def cb_pose(self, msg: Point):
        self.N, self.E, self.D = msg.x, msg.y, msg.z

    def cb_att(self, msg: Vector3):
        # roll, pitch, yaw (rad). We need yaw only here.
        self.psi = msg.z

    def cb_vel(self, msg: Float32):
        self.V = float(msg.data)

    def cb_target(self, msg: Point):
        self.target_n, self.target_e, self.target_d = msg.x, msg.y, msg.z
        self.get_logger().info(f'New target NED: N={self.target_n:.1f} E={self.target_e:.1f} D={self.target_d:.1f}')

    def step(self):
        # publish current target at low rate (e.g., 5 Hz)
        if int(self.get_clock().now().nanoseconds * 1e-9 * 5) % 1 == 0:
            p = Point(); p.x=self.target_n; p.y=self.target_e; p.z=self.target_d
            self.tgt_pub.publish(p)

        # Params that affect dynamics
        k_nav = float(self.get_parameter('k_nav').value)
        theta0_deg = float(self.get_parameter('theta0_deg').value)
        kz = float(self.get_parameter('kz').value)
        kv = float(self.get_parameter('kv').value)
        V_ref = float(self.get_parameter('V_ref').value)
        phi_max_deg = float(self.get_parameter('phi_max_deg').value)
        theta_min_deg = float(self.get_parameter('theta_min_deg').value)
        theta_max_deg = float(self.get_parameter('theta_max_deg').value)

        eN = self.target_n - self.N
        eE = self.target_e - self.E
        psi_des = math.atan2(eE, eN)
        psi_err = wrap_angle(psi_des - self.psi)

        a_y = k_nav * self.V * math.sin(psi_err)
        phi_des = math.atan2(a_y, 9.80665)
        phi_des = clamp(phi_des, -math.radians(phi_max_deg), math.radians(phi_max_deg))

        theta0 = math.radians(theta0_deg)
        theta_des = theta0 + kz * (self.target_d - self.D) + kv * (V_ref - self.V)
        theta_des = clamp(theta_des, math.radians(theta_min_deg), math.radians(theta_max_deg))

        sp = Vector3()
        sp.x = phi_des   # roll setpoint
        sp.y = theta_des # pitch setpoint
        sp.z = 0.0
        self.sp_pub.publish(sp)

def main():
    rclpy.init()
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
