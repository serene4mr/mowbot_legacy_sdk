#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ARGS = [
    
    # Launch arguments for multi-model setup
    
    DeclareLaunchArgument(
        'model',
        default_value='mowbot',
        description='Robot model to load (URDF/Xacro filename without extension, e.g., amr1, amr2)'
    ),
    
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace of the robot (useful in multi-robot/multi-AMR scenarios)'
    ),
    
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
]

def generate_launch_description():
    package_name = 'mowbot_description'

    # Launch configurations
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Dynamic path to Xacro file based on model argument
    urdf_file = PathJoinSubstitution(
        [FindPackageShare(package_name), 'urdf', model, '.urdf.xacro']
    )

    # Info log for clarity
    log_info = LogInfo(
        msg=['[mowbot_description] Loading model: ', model, ', file: ', urdf_file]
    )

    # Xacro command with argument passthrough
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            urdf_file,
            ' ',
            'is_sim:=', use_sim_time
        ]),
        value_type=str
    )

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription(ARGS + [
        log_info,
        robot_state_publisher_node
    ])
