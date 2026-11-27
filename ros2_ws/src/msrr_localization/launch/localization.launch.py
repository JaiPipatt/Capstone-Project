"""
AprilTag localization of the real modules (overhead USB webcam), optionally with RViz.

    ros2 launch msrr_localization localization.launch.py

config:=<file>  parameter file (default config/tag_pose.yaml: camera device, tag map)
rviz:=false     no RViz
fusion:=false   no encoder fusion (fusion_node, config/fusion.yaml)
fusion_config:=<file>  fusion_node parameter file
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
        DeclareLaunchArgument('fusion', default_value='true'),
        DeclareLaunchArgument('fusion_config', default_value=PathJoinSubstitution(
            [share, 'config', 'fusion.yaml'])),
        Node(
            package='msrr_localization',
            executable='tag_pose_node',
            parameters=[LaunchConfiguration('config')],
            output='screen'
        ),
        Node(
            package='msrr_localization',
            executable='fusion_node',
            parameters=[LaunchConfiguration('fusion_config')],
            condition=IfCondition(LaunchConfiguration('fusion')),
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
