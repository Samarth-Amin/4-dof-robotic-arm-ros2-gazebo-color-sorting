"""Bring up Scout in Gazebo Harmonic: world + robot + ros_gz bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('scout_description')

    world_file = os.path.join(pkg, 'worlds', 'scout_world.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'scout.urdf.xacro')
    bridge_file = os.path.join(pkg, 'config', 'gz_bridge.yaml')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    gz_sim_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py',
        )
    )

    # gui:=true runs the full client + server; gui:=false runs the server only
    # (`-s`), which is what CI and headless smoke tests need.
    gazebo = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={
            'gz_args': ['-r -v 3 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui),
    )

    gazebo_headless = IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={
            'gz_args': ['-r -v 3 -s ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'scout',
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_file,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg, 'worlds'),
        ),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='Run the Gazebo GUI'
        ),
        DeclareLaunchArgument('x', default_value='-4.5'),
        DeclareLaunchArgument('y', default_value='-3.5'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.9'),

        gazebo,
        gazebo_headless,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=3.0, actions=[bridge]),
    ])