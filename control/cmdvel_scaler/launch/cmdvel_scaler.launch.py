from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Declare launch arguments
    declared_args = [
        DeclareLaunchArgument(
            'use_params_file',
            default_value='true',
            description='Whether to use params file or individual parameters'
        ),
        DeclareLaunchArgument(
            'cmdvel_scaler_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('cmdvel_scaler'), 'config', 'cmdvel_scaler_params.yaml']
            ),
            description='Path to the cmdvel_scaler parameters file'
        ),
        DeclareLaunchArgument(
            'left_rate',
            default_value='1.0',
            description='Left wheel rate'
        ),
        DeclareLaunchArgument(
            'right_rate',
            default_value='1.0',
            description='Right wheel rate'
        ),
        DeclareLaunchArgument(
            'wheel_separation',
            default_value='0.5',
            description='Wheel separation'
        ),
    ]

    # Create two nodes with different conditions and parameter sources
    node_with_file = Node(
        package='cmdvel_scaler',
        executable='cmdvel_scaler_node',
        name='cmdvel_scaler_node',
        output='screen',
        parameters=[LaunchConfiguration('cmdvel_scaler_params_file')],
        condition=IfCondition(LaunchConfiguration('use_params_file'))
    )
    
    node_with_params = Node(
        package='cmdvel_scaler',
        executable='cmdvel_scaler_node',
        name='cmdvel_scaler_node',
        output='screen',
        parameters=[{
            'left_rate': LaunchConfiguration('left_rate'),
            'right_rate': LaunchConfiguration('right_rate'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
        }],
        condition=UnlessCondition(LaunchConfiguration('use_params_file'))
    )

    return LaunchDescription(declared_args + [
        node_with_file,
        node_with_params
    ])
