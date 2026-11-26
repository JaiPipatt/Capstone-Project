"""
Run the algorithm against the simulation.

Terminal 1: ros2 launch my_robot_description sim.launch.py num_modules:=3
Terminal 2: ros2 launch msrr_control algo.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('msrr_control'), 'config', 'algo.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='msrr_control',
            executable='algo_node',
            parameters=[params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])
