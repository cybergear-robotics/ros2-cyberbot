from launch import LaunchDescription
from launch_ros.actions import Node
from math import radians

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyAMA2", "--baudrate", "921600"]
        ),
    ])
