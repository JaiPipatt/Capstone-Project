"""Spawn N identical modules in Gazebo Fortress.

    ros2 launch my_robot_description sim.launch.py num_modules:=3 gui:=true

Each module i (1..N) lives in namespace module_i with its own
robot_state_publisher, controller_manager and controllers.
Modules start in a row along y, spaced by `spacing` metres.
"""
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def spawn_modules(context):
    pkg_path = FindPackageShare('my_robot_description')
    xacro_file = PathJoinSubstitution([pkg_path, 'urdf', 'my_robot.urdf.xacro'])

    num_modules = int(LaunchConfiguration('num_modules').perform(context))
    spacing = float(LaunchConfiguration('spacing').perform(context))

    actions = []
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    bridge_remaps = []

    for i in range(1, num_modules + 1):
        ns = f'module_{i}'
        y = (i - 1 - (num_modules - 1) / 2) * spacing  # centered row

        robot_description = ParameterValue(
            Command(['xacro ', xacro_file, f' namespace:={ns}']),
            value_type=str
        )

        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': f'{ns}/',
                'use_sim_time': True
            }],
            output='screen'
        ))

        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', f'/{ns}/robot_description', '-name', ns,
                       '-x', '0', '-y', str(y), '-z', '0.01'],
            output='screen'
        )
        actions.append(spawn)

        # Controllers start only after this module exists in Gazebo
        controllers = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['-c', f'/{ns}/controller_manager',
                       'joint_state_broadcaster',
                       'diff_drive_controller',
                       'head_position_controller'],
            output='screen'
        )
        actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers])))

        # Ground truth from Gazebo -> /module_i/ground_truth/odom and /tf
        bridge_args += [
            f'/model/{ns}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/model/{ns}/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ]
        bridge_remaps += [
            (f'/model/{ns}/odometry', f'/{ns}/ground_truth/odom'),
            (f'/model/{ns}/pose', '/tf'),
        ]

    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remaps,
        output='screen'
    ))
    return actions


def generate_launch_description():

    pkg_path = FindPackageShare('my_robot_description')

    # Let Gazebo resolve package://my_robot_description/... mesh paths
    resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        PathJoinSubstitution([pkg_path, '..'])
    )

    gz_sim_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    # Camera placed close to the robots (default view is 6 m away)
    gui_config = PathJoinSubstitution([pkg_path, 'config', 'gui.config'])

    # -r: start playing immediately, -s: server only (no window)
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r empty.sdf --gui-config ', gui_config]}.items(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': '-r -s empty.sdf'}.items(),
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('num_modules', default_value='3', description='Number of modules to spawn'),
        DeclareLaunchArgument('spacing', default_value='0.3', description='Distance between modules [m]'),
        resource_path,
        gazebo_gui,
        gazebo_headless,
        OpaqueFunction(function=spawn_modules),
    ])
