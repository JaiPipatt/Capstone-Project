"""
Start the simulation and drive one module with the keyboard.

    ros2 launch msrr_control teleop.launch.py module:=module_1 num_modules:=3

sim:=false skips Gazebo (e.g. to drive the real robot: start the micro-ROS
agent, see communcation/README.md, and the module ESP32s subscribe directly).
teleop_twist_keyboard needs a TTY, so it opens in its own xterm window;
keep that window focused while pressing keys.
Do not run alongside algo.launch.py: both publish to the same cmd_vel topic.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    module = LaunchConfiguration('module')
    sim = LaunchConfiguration('sim')

    sim_launch = PathJoinSubstitution([FindPackageShare('my_robot_description'), 'launch', 'sim.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('module', default_value='module_1', description='Module to drive'),
        DeclareLaunchArgument('sim', default_value='true', description='Start the Gazebo simulation'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('num_modules', default_value='3', description='Number of modules to spawn'),
        DeclareLaunchArgument('spacing', default_value='0.3', description='Distance between modules [m]'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'num_modules': LaunchConfiguration('num_modules'),
                'spacing': LaunchConfiguration('spacing'),
            }.items(),
            condition=IfCondition(sim)
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name=['teleop_', module],
            prefix='xterm -e',
            remappings=[('cmd_vel', ['/', module, '/diff_drive_controller/cmd_vel_unstamped'])],
            parameters=[{'use_sim_time': sim}],
            output='screen'
        ),
    ])
