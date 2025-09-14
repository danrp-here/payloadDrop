import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
def clamp(x,lo,hi): return max(lo,min(hi,x))
class Mixer4Node(Node):
    def __init__(self):
        super().__init__('mixer4_node')
        self.declare_parameter('limit_servo', 1.0)
        self.create_subscription(Float32MultiArray, '/cmd/attitude_u', self.cb_u, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/cmd/surfaces_norm_4', 10)
        self.get_logger().info('mixer4_node (+ cruciform) started')
    def cb_u(self,msg:Float32MultiArray):
        limit=float(self.get_parameter('limit_servo').value)
        if len(msg.data)<2: return
        u_phi,u_th=float(msg.data[0]),float(msg.data[1])
        s1 = clamp(+u_phi, -limit, +limit)  # Left (West)
        s2 = clamp(-u_phi, -limit, +limit)  # Right (East)
        s3 = clamp(+u_th , -limit, +limit)  # Front/Top (North)
        s4 = clamp(-u_th , -limit, +limit)  # Back/Bottom (South)
        out=Float32MultiArray(); out.data=[s1,s2,s3,s4]
        self.pub.publish(out)
def main():
    rclpy.init(); n=Mixer4Node(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
