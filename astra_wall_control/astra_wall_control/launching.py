"""Shared isolated SITL stack for the three user-facing launch files."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def scenario(mode):
    share = get_package_share_directory('astra_wall_control')
    args = [DeclareLaunchArgument(name, default_value=value) for name, value in (
        ('gui', 'true'), ('rviz', 'true'), ('auto_arm', 'true'),
        ('goal_height', '1.4'), ('takeoff_height', '1.6'), ('lidar_downsample', '4'),
        ('hold_body_motion', 'true' if mode == 'wholebody' else 'false'), ('body_motion_period', '32.0'),
        ('body_motion_bounds', '[0.06, 0.06, 0.05, 0.20]'),
        ('body_motion_amplitude', '[0.025, 0.035, 0.02, 0.08]'),
        ('corridor_file', os.path.join(share, 'config/corridor.json')))]
    args.extend([
        DeclareLaunchArgument('dashboard', default_value=LaunchConfiguration('rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch/wall.launch.py')),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in
                   ('gui', 'auto_arm', 'goal_height', 'takeoff_height', 'lidar_downsample', 'corridor_file', 'hold_body_motion',
                    'body_motion_period', 'body_motion_bounds', 'body_motion_amplitude')},
                'world': os.path.join(share, 'worlds/corridor.world' if mode == 'corridor' else 'worlds/air.world'),
                'interactive_goal': 'true', 'control_mode': mode,
            }.items()),
        Node(package='astra_wall_control', executable='air_goal.py', output='screen',
             parameters=[{'use_sim_time': True, 'goal_kind': 'ee' if mode == 'wholebody' else 'body',
                          'control_mode': mode,
                          'scene_file': os.path.join(share, 'worlds/corridor.world' if mode == 'corridor' else 'worlds/air.world')}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'config/air_test.rviz' if mode == 'corridor' else 'config/manipulation.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='astra_wall_control', executable='flight_dashboard.py', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('dashboard'))),
    ])
    if mode == 'manual':
        args.extend([
            DeclareLaunchArgument('sliders', default_value='true'),
            Node(package='astra_wall_control', executable='manual_sliders.py', output='screen',
                 parameters=[{'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('sliders'))),
        ])
    return LaunchDescription(args)
