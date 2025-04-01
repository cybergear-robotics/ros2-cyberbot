from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/sllidar.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/agent.launch.py'])),
#        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/teleop.launch.py'])),
#        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/navigation.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/d435.launch.py'])),
    ])
