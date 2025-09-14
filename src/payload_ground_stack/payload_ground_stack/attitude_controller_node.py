import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def wrap_angle(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a

class AttitudeControllerNode(Node):
    def __init__(self):
        super().__init__('attitude_controller_node')
        self.declare_parameter('kp_roll', 0.8)
        self.declare_parameter('ki_roll', 0.0)
        self.declare_parameter('kd_roll', 0.1)
        self.declare_parameter('kp_pitch', 0.8)
        self.declare_parameter('ki_pitch', 0.0)
        self.declare_parameter('kd_pitch', 0.1)
        self.declare_parameter('i_limit', 0.5)
        self.phi=self.theta=0.0; self.p=self.q=0.0; self.phi_des=self.theta_des=0.0
        self.i_phi=self.i_th=0.0
        self.create_subscription(Vector3,'/state/attitude',self.cb_att,10)
        self.create_subscription(Vector3,'/state/rates',self.cb_rates,10)
        self.create_subscription(Vector3,'/cmd/attitude_sp',self.cb_sp,10)
        self.pub_u = self.create_publisher(Float32MultiArray,'/cmd/attitude_u',10)
        self.dt=0.02; self.timer=self.create_timer(self.dt, self.step)
        self.get_logger().info('attitude_controller_node started')
    def cb_att(self,msg:Vector3): self.phi,self.theta=msg.x,msg.y
    def cb_rates(self,msg:Vector3): self.p,self.q=msg.x,msg.y
    def cb_sp(self,msg:Vector3): self.phi_des,self.theta_des=msg.x,msg.y
    def step(self):
        p = self.get_parameters(['kp_roll','ki_roll','kd_roll','kp_pitch','ki_pitch','kd_pitch','i_limit'])
        kp_r,ki_r,kd_r,kp_t,ki_t,kd_t,i_lim = [pp.value for pp in p]
        e_phi = wrap_angle(self.phi_des - self.phi); e_th = self.theta_des - self.theta
        self.i_phi = clamp(self.i_phi + e_phi*self.dt, -i_lim, i_lim)
        self.i_th  = clamp(self.i_th  + e_th *self.dt, -i_lim, i_lim)
        u_phi = kp_r*e_phi + ki_r*self.i_phi - kd_r*self.p
        u_th  = kp_t*e_th  + ki_t*self.i_th  - kd_t*self.q
        msg = Float32MultiArray(); msg.data=[float(u_phi), float(u_th)]; self.pub_u.publish(msg)

def main():
    rclpy.init(); node=AttitudeControllerNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
