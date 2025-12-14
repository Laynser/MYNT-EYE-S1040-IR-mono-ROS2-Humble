from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mynt_eye',
            executable='depth_imu_publisher',
            name='depth_imu_publisher',
            namespace='mynt_up',
            arguments=['000000412E1700090913'],
            parameters=[{
                'publish_left': False,
                'publish_right': False,
                'publish_depth': False,
                'publish_points': False,
                'publish_imu': False,
            }],
            output='screen'
        ),
    ])
