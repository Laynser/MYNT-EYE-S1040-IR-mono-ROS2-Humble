from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mynt_eye',
            executable='depth_imu_publisher',
            name='depth_imu_publisher',
            namespace='mynt_0',
            parameters=[{
                'serial_number': '000000412E1700090913',
                'publish_left': False,
                'publish_right': False,
                'publish_depth': False,
                'publish_points': True,
                'publish_imu': False,
            }],
            output='screen'
        ),
    ])
