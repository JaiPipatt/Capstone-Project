"""
Self-assembly, simulation or real modules, chosen with one argument.

    ros2 launch msrr_control algo.launch.py robot:=sim     # Gazebo, poses from ground truth
    ros2 launch msrr_control algo.launch.py robot:=real    # real modules, poses from AprilTags

robot:=sim   includes algo_sim.launch.py   (config/algo_sim.yaml,  /module_i/ground_truth/odom)
robot:=real  includes algo_real.launch.py  (config/algo_real.yaml, /module_i/tag/odom)
Every other argument is passed through to that file, e.g.
    robot:=sim num_modules:=3 gui:=false     robot:=real algo:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

TARGETS = {'sim': 'algo_sim.launch.py', 'real': 'algo_real.launch.py'}


def include_target(context):
    robot = context.launch_configurations['robot'].strip().lower()
    if robot not in TARGETS:
        raise RuntimeError(f"robot:='{robot}' is not valid, use robot:=sim or robot:=real")
    target = PathJoinSubstitution([FindPackageShare('msrr_control'), 'launch', TARGETS[robot]])
    # launch arguments given on the command line are visible to the included file
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(target))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', choices=['sim', 'real'],
                              description='sim = Gazebo ground truth, real = AprilTag poses'),
        OpaqueFunction(function=include_target),
    ])
