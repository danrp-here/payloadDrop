import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
def clamp(x,lo,hi): return max(lo,min(hi,x))
class MixerNode(Node):
    def __init__(self):
        super().__init__('mixer_node')
        self.declare_parameter('limit_servo', 1.0)
        self.create_subscription(Float32MultiArray, '/cmd/attitude_u', self.cb_u, 10)
        self.pub_vec = self.create_publisher(Vector3, '/cmd/surfaces_norm_vec', 10)
        self.get_logger().info('mixer_node (2-servo elevon) started')
    def cb_u(self,msg:Float32MultiArray):
        limit=float(self.get_parameter('limit_servo').value)
        if len(msg.data)<2: return
        u_phi,u_th=float(msg.data[0]),float(msg.data[1])
        left  = clamp(u_th + u_phi, -limit, +limit)
        right = clamp(u_th - u_phi, -limit, +limit)
        out=Vector3(); out.x=left; out.y=right; out.z=0.0
        self.pub_vec.publish(out)
def main():
    rclpy.init(); n=MixerNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
