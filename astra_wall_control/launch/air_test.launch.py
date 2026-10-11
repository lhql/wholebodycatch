"""Independent SITL air test with RViz end-effector goals."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('astra_wall_control')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_arm', default_value='true'),
        DeclareLaunchArgument('goal_height', default_value='1.4'),
        DeclareLaunchArgument('lidar_downsample', default_value='4'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch/wall.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'auto_arm': LaunchConfiguration('auto_arm'),
                'goal_height': LaunchConfiguration('goal_height'),
                'lidar_downsample': LaunchConfiguration('lidar_downsample'),
                'world': os.path.join(share, 'worlds/air.world'),
                'interactive_goal': 'true',
            }.items()),
        Node(package='astra_wall_control', executable='air_goal.py', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'config/air_test.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
