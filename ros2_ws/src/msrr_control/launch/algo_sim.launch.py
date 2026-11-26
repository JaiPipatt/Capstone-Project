"""
Self-assembly in Gazebo, with RViz showing the plan.

    ros2 launch msrr_control algo_sim.launch.py num_modules:=3

gazebo:=false  only the controller (+ RViz); start Gazebo yourself with
               ros2 launch my_robot_description sim.launch.py num_modules:=3
rviz:=false    no RViz
gui:=false     headless Gazebo
The target shape is `connections` in config/algo_sim.yaml; num_modules must fit it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('msrr_control')
    params = PathJoinSubstitution([share, 'config', 'algo_sim.yaml'])
    sim_launch = PathJoinSubstitution(
        [FindPackageShare('my_robot_description'), 'launch', 'sim.launch.py'])
    num_modules = LaunchConfiguration('num_modules')

    return LaunchDescription([
        DeclareLaunchArgument('num_modules', default_value='3'),
        DeclareLaunchArgument('spacing', default_value='0.3', description='start spacing [m]'),
        DeclareLaunchArgument('gazebo', default_value='true', description='start Gazebo'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'num_modules': num_modules,
                'spacing': LaunchConfiguration('spacing'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('gazebo'))
        ),
        Node(
            package='msrr_control',
            executable='algo_node',
            parameters=[params, {
                'use_sim_time': True,
                'num_modules': ParameterValue(num_modules, value_type=int),
            }],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'sim.rviz'])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        ),
    ])
