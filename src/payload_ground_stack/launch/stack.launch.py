from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='payload_ground_stack', executable='payload_mav_bridge', output='screen', name='bridge',
            parameters=[{
                'conn_url': '/dev/ttyUSB1:57600',
                'payload_sysid': 72,
                'servo1': 1, 'servo2': 2, 'servo3': 3, 'servo4': 4,
                'pwm_center': 1500,
                'pwm_scale': 400,
            }]
        ),
        Node(
            package='payload_ground_stack', executable='guidance_node', output='screen', name='guidance',
            parameters=[{
                'target_n': 0.0,
                'target_e': 300.0,
                'target_d': 0.0,
                'k_nav': 0.9,
                'theta0_deg': -2.0,
                'kz': 0.003,
                'kv': 0.02,
                'V_ref': 15.0,
                'phi_max_deg': 35.0,
                'theta_min_deg': -10.0,
                'theta_max_deg': 10.0,
            }]
        ),
        Node(
            package='payload_ground_stack', executable='attitude_controller_node', output='screen', name='att_ctrl',
            parameters=[{
                'kp_roll': 0.8, 'kd_roll': 0.1, 'ki_roll': 0.0,
                'kp_pitch': 0.8, 'kd_pitch': 0.1, 'ki_pitch': 0.0,
                'i_limit': 0.5,
            }]
        ),
        Node(
            package='payload_ground_stack', executable='mixer4_node', output='screen', name='mixer',
            parameters=[{'limit_servo': 1.0}]
        ),
    ])
