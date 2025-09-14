import rclpy, math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
def clamp(x,lo,hi): return max(lo,min(hi,x))
K = 1.0/math.sqrt(2.0)
class Mixer4NodeX(Node):
    def __init__(self):
        super().__init__('mixer4_node_x')
        self.declare_parameter('limit_servo', 1.0)
        self.create_subscription(Float32MultiArray, '/cmd/attitude_u', self.cb_u, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/cmd/surfaces_norm_4', 10)
        self.get_logger().info('mixer4_node_x (× cruciform) started')
    def cb_u(self,msg:Float32MultiArray):
        limit=float(self.get_parameter('limit_servo').value)
        if len(msg.data)<2: return
        u_phi,u_th=float(msg.data[0]),float(msg.data[1])
        s1 = clamp(K*(+u_th + u_phi), -limit, +limit)  # Front-Left (NW)
        s2 = clamp(K*(+u_th - u_phi), -limit, +limit)  # Front-Right (NE)
        s3 = clamp(K*(-u_th - u_phi), -limit, +limit)  # Rear-Right (SE)
        s4 = clamp(K*(-u_th + u_phi), -limit, +limit)  # Rear-Left (SW)
        out=Float32MultiArray(); out.data=[s1,s2,s3,s4]
        self.pub.publish(out)
def main():
    rclpy.init(); n=Mixer4NodeX(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
