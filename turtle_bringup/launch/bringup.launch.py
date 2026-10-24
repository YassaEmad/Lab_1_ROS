import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('turtle_bringup')
    params_file = os.path.join(share, 'config', 'turtle_params.yaml')

    linear_arg = DeclareLaunchArgument('max_linear_speed', default_value='2.0')
    angular_arg = DeclareLaunchArgument('max_angular_speed', default_value='2.0')

    speed_params = {
        'max_linear_speed': ParameterValue(
            LaunchConfiguration('max_linear_speed'), value_type=float),
        'max_angular_speed': ParameterValue(
            LaunchConfiguration('max_angular_speed'), value_type=float),
    }

    return LaunchDescription([
        linear_arg,
        angular_arg,
        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        Node(package='turtle_control', executable='go_to_pose_server',
             name='go_to_pose_server', output='screen',
             parameters=[params_file, speed_params]),
        Node(package='turtle_control', executable='reset_service',
             name='reset_service', output='screen'),
        Node(package='turtle_control', executable='draw_square_server',
             name='draw_square_server', output='screen',
             parameters=[params_file]),
    ])
