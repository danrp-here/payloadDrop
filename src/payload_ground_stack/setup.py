from setuptools import setup

package_name = 'payload_ground_stack'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/stack.launch.py', 'launch/stack_2servo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Ground-side MAVLink bridge and controllers (ament_python)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'payload_mav_bridge = payload_ground_stack.payload_mav_bridge:main',
            'guidance_node = payload_ground_stack.guidance_node:main',
            'attitude_controller_node = payload_ground_stack.attitude_controller_node:main',
            'mixer_node = payload_ground_stack.mixer_node:main',
            'mixer4_node = payload_ground_stack.mixer4_node:main',
            'mixer4_node_x = payload_ground_stack.mixer4_node_x:main',
        ],
    },
)
