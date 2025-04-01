from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from os import path
from launch import LaunchDescription



def generate_launch_description():
    return LaunchDescription([
        Node(
            name='joy_node',
            package='joy',
            executable='joy_node',
            parameters=[
                path.join(get_package_share_directory('cyberbot_bringup'), 'config', 'joy.yaml'),
            ]
        ),
        Node(
            name='teleop_twist_joy_node',
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[
                path.join(get_package_share_directory('cyberbot_bringup'), 'config', 'teleop.yaml'),
            ]
        ),
    ])
