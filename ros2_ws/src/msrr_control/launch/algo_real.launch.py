"""
Self-assembly on the real modules, localized by the overhead AprilTag camera, with
RViz showing the plan and the tag poses.

Before: hotspot and micro-ROS agent running (communcation/README.md), modules on,
floor calibrated (ros2 run msrr_localization tag_calibrate --help).

    ros2 launch msrr_control algo_real.launch.py

algo:=false        localization + RViz only, the modules do not move (check tags first)
rviz:=false        no RViz
tag_config:=<file> tag_pose_node parameters (camera IP, tag map); default from
                   msrr_localization/config/tag_pose.yaml
The target shape is `connections` in config/algo_real.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('msrr_control')
    loc_share = FindPackageShare('msrr_localization')
    params = PathJoinSubstitution([share, 'config', 'algo_real.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('algo', default_value='true', description='run the controller'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('tag_config', default_value=PathJoinSubstitution(
            [loc_share, 'config', 'tag_pose.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([loc_share, 'launch', 'localization.launch.py'])),
            launch_arguments={'config': LaunchConfiguration('tag_config'),
                              'rviz': 'false'}.items(),
        ),
        Node(
            package='msrr_control',
            executable='algo_node',
            parameters=[params, {'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('algo')),
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'real.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        ),
    ])
