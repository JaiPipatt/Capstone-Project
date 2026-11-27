"""
Self-assembly planner/controller on the real modules (algo_node only).

Run localization separately first; it also opens RViz, which shows the plan too:

    # Terminal 1: camera + encoder fusion + RViz
    ros2 launch msrr_localization localization.launch.py
    # Terminal 2: the planner
    ros2 launch msrr_control algo_real.launch.py

Before: hotspot and micro-ROS agent running (communcation/README.md), modules on.
Poses come from /module_i/fused/odom (pose_topic in config/algo_real.yaml).
The target shape is `connections` in config/algo_real.yaml.

params:=<file>  algo_node parameters (default config/algo_real.yaml)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('msrr_control')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
            [share, 'config', 'algo_real.yaml'])),
        Node(
            package='msrr_control',
            executable='algo_node',
            parameters=[LaunchConfiguration('params'), {'use_sim_time': False}],
            output='screen'
        ),
    ])
