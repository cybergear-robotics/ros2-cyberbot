"""Launch realsense2_camera node."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os import path

def generate_launch_description():    
    launch_dir = path.join(get_package_share_directory('realsense2_camera'), 'launch')
    return LaunchDescription([
        Node(
            package='tf2_ros', 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0.43", "0", "0", "0", "base_link", "camera_link"]
        ),
        TimerAction(
            period=0.5,            
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([launch_dir, '/rs_launch.py']),
                    launch_arguments=[("enable_pointcloud", "true")]
                ),
           ]
        ),
    ])
