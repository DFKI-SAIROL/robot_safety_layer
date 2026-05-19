from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments for customization
        DeclareLaunchArgument(
            'arm_prefix',
            default_value='franka_left',
            description='Namespace and prefix for the robot (e.g. franka_left)'
        ),
        DeclareLaunchArgument(
            'bypass_safety',
            default_value='true',
            description='If true, safety checks are bypassed'
        ),
        DeclareLaunchArgument(
            'safety_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_safety_layer'), 'config', 'safety_params.yaml'
            ]),
            description='Path to the safety parameters config file'
        ),

        # The Safety Node
        Node(
            package='robot_safety_layer',
            executable='franka_controller_node',
            name='safety_node',
            namespace=LaunchConfiguration('arm_prefix'),
            parameters=[
                LaunchConfiguration('safety_config_file'),
                {
                    'arm_prefix': LaunchConfiguration('arm_prefix'),
                    'bypass_safety': LaunchConfiguration('bypass_safety'),
                }
            ],
            output='screen',
        )
    ])
