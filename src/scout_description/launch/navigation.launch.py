"""Full stack: Gazebo world + Scout + Nav2.

Two modes, selected by the `slam` argument:

  slam:=true   slam_toolbox builds the map online while Nav2 plans on it
  slam:=false  map_server loads `map:=<yaml>` and AMCL localizes on it

In both modes only ONE map -> odom publisher exists, which is what keeps
the TF tree sane.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('scout_description')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    rviz_file = os.path.join(pkg, 'rviz', 'nav.rviz')
    default_map = os.path.join(pkg, 'maps', 'scout_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')

    # ---- Gazebo world + robot + ros_gz bridge -------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'gui': gui,
        }.items(),
    )

    # ---- mapping mode -------------------------------------------------
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            slam_params,
            {
                'use_lifecycle_manager': False,
                'use_sim_time': use_sim_time,
            },
        ],
        condition=IfCondition(slam),
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(slam),
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(slam),
    )

    # ---- localization mode (map_server + AMCL) ------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': use_sim_time,
            'topic_name': 'map',
            'frame_id': 'map',
        }],
        condition=UnlessCondition(slam),
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan',
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'set_initial_pose': True,
            'initial_pose': {
                'x': -4.5, 'y': -3.5, 'z': 0.0,
                'yaw': 0.9,
            },
        }],
        condition=UnlessCondition(slam),
    )

    # lifecycle manager that brings map_server + amcl up together
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=UnlessCondition(slam),
    )

    # ---- Nav2 servers -------------------------------------------------
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params,
                    'autostart': 'true',
                    'use_composition': 'False',
                }.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_file],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false = run the Gazebo server only (headless)',
        ),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='true = map online with slam_toolbox, '
                        'false = localize on a saved map',
        ),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('rviz', default_value='true'),

        gazebo,
        slam_toolbox,
        slam_configure,
        slam_activate,
        map_server,
        amcl,
        localization_lifecycle,
        nav2,
        rviz,
    ])