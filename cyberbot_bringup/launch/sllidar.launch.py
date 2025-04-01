from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from os import path
from launch import LaunchDescription
from math import radians

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='sllidar',
            package='sllidar_ros2',
            executable='sllidar_node',
            output='screen',
            parameters=[
                path.join(get_package_share_directory('cyberbot_bringup'), 'config', 'sllidar.yaml'),
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0.0", "0.0", "0.43" ,str(radians(180)), "0.0", "0.0" ,'base_link','laser'],
            
        )
    ])
