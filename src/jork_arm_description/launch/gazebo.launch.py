import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('jork_arm_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    xacro_file = os.path.join(pkg, 'urdf', 'jork_arm.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'sorting_cell.sdf')
    rviz_file = os.path.join(pkg, 'rviz', 'view_arm.rviz')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=str(Path(pkg).parent.resolve())
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'jork_arm',
            '-topic', 'robot_description',
            '-z', '0.0',
        ],
    )

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/jork_arm/joint_1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/jork_arm/joint_2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/jork_arm/joint_3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/jork_arm/joint_4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/jork_arm/finger_left/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/jork_arm/finger_right/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
    ]
    for pid in range(1, 16):
        bridge_args.append(f'/conveyor/puck_{pid}/cmd_x@std_msgs/msg/Float64]gz.msgs.Double')
        bridge_args.append(f'/conveyor/puck_{pid}/cmd_y@std_msgs/msg/Float64]gz.msgs.Double')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_args,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        gz_resource_path,
        gazebo_gui,
        robot_state_publisher,
        spawn,
        bridge,
        rviz_node,
    ])
