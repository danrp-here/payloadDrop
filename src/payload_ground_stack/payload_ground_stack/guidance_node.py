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
        self.N=self.E=self.D=0.0; self.psi=0.0; self.V=10.0
        self.create_subscription(Point,'/state/pose_ned',self.cb_pose,10)
        self.create_subscription(Vector3,'/state/attitude',self.cb_att,10)
        self.create_subscription(Float32,'/state/vel',self.cb_vel,10)
        self.sp_pub = self.create_publisher(Vector3,'/cmd/attitude_sp',10)
        self.timer = self.create_timer(0.02, self.step)
        self.get_logger().info('guidance_node started')
    def cb_pose(self,msg:Point): self.N,self.E,self.D=msg.x,msg.y,msg.z
    def cb_att(self,msg:Vector3): self.psi=msg.z
    def cb_vel(self,msg:Float32): self.V=float(msg.data)
    def step(self):
        p = self.get_parameters(['target_n','target_e','target_d','k_nav','theta0_deg','kz','kv','V_ref','phi_max_deg','theta_min_deg','theta_max_deg'])
        target_n, target_e, target_d, k_nav, theta0_deg, kz, kv, V_ref, phi_max_deg, theta_min_deg, theta_max_deg = [pp.value for pp in p]
        eN = target_n - self.N; eE = target_e - self.E
        psi_des = math.atan2(eE, eN); psi_err = wrap_angle(psi_des - self.psi)
        a_y = k_nav * self.V * math.sin(psi_err)
        phi_des = math.atan2(a_y, 9.80665); phi_des = clamp(phi_des, -math.radians(phi_max_deg), math.radians(phi_max_deg))
        theta0 = math.radians(theta0_deg)
        theta_des = theta0 + kz*(target_d - self.D) + kv*(V_ref - self.V)
        theta_des = clamp(theta_des, math.radians(theta_min_deg), math.radians(theta_max_deg))
        sp = Vector3(); sp.x=phi_des; sp.y=theta_des; sp.z=0.0; self.sp_pub.publish(sp)

def main():
    rclpy.init(); node=GuidanceNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
