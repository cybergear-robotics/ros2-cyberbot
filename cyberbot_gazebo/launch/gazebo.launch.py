import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = True

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )
    
    world_path = PathJoinSubstitution(
        [FindPackageShare("cyberbot_gazebo"), "worlds", "playground.sdf"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('cyberbot_description'), 'launch', 'description.launch.py']
    )
    gui_config_path = PathJoinSubstitution(
        [FindPackageShare("cyberbot_gazebo"), "config", "gui.config"]
    )
    ros_gz_bridge_config_path = PathJoinSubstitution(
        [FindPackageShare("cyberbot_gazebo"), "config", "ros_gz_bridge.yaml"]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("cyberbot_gazebo"), "config", "cyberbot_controllers.yaml"]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='headless', 
            default_value='true',
            description='Disable Gazebo GUI'
        ),
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        DeclareLaunchArgument(
            name='spawn_x', 
            default_value='0.0',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y', 
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z', 
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
            
        DeclareLaunchArgument(
            name='spawn_yaw', 
            default_value='0.0',
            description='Robot spawn heading'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'gz_args': [
                    LaunchConfiguration('world'),
                    " -r", # run on start
                    #" -s", # headless (overwites -g)
                    " -v 6",
                    ' --gui-config ',
                    gui_config_path
                ],
                'on_exit_shutdown': 'True'
            }.items()
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', 'cyberbot', 
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{
                'config_file': ros_gz_bridge_config_path,
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'cyberbot_controller',
                '--param-file', robot_controllers,
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--param-file', robot_controllers,
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
            }.items()
        )

    ])
