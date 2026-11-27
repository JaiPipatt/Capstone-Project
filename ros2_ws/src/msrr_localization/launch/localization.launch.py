"""
AprilTag localization of the real modules (overhead USB webcam), optionally with RViz.

    ros2 launch msrr_localization localization.launch.py

config:=<file>  parameter file (default config/tag_pose.yaml: camera device, tag map)
rviz:=false     no RViz
Calibrate the floor first: ros2 run msrr_localization tag_calibrate --help
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('msrr_localization')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [share, 'config', 'tag_pose.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='msrr_localization',
            executable='tag_pose_node',
            parameters=[LaunchConfiguration('config')],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'localization.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        ),
    ])
