import os
from typing import List
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=get_package_share_directory('astra_wall_control')
    runner=os.path.join(get_package_prefix('astra_wall_control'),'lib/astra_wall_control/run_sim.sh')
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='false'),
        DeclareLaunchArgument('world',default_value=os.path.join(share,'worlds/wall.world')),
        DeclareLaunchArgument('interactive_goal',default_value='false'),
        DeclareLaunchArgument('control_mode',default_value='wholebody'),
        DeclareLaunchArgument('hold_body_motion',default_value='false'),
        DeclareLaunchArgument('body_motion_period',default_value='32.0'),
        DeclareLaunchArgument('body_motion_bounds',default_value='[0.06, 0.06, 0.05, 0.20]'),
        DeclareLaunchArgument('body_motion_amplitude',default_value='[0.025, 0.035, 0.02, 0.08]'),
        DeclareLaunchArgument('takeoff_height',default_value='1.6'),
        DeclareLaunchArgument('corridor_file',default_value=os.path.join(share,'config/corridor.json')),
        DeclareLaunchArgument('goal_height',default_value='1.4'),
        DeclareLaunchArgument('auto_arm',default_value='true'),
        DeclareLaunchArgument('lidar_downsample',default_value='4'),
        ExecuteProcess(cmd=[runner,'my',LaunchConfiguration('world'),LaunchConfiguration('gui'),share,LaunchConfiguration('lidar_downsample')],output='screen'),
        Node(package='mavros',executable='mavros_node',namespace='mavros',name='mavros',output='screen',parameters=[{
            'use_sim_time':True,'fcu_url':'udp://:14540@localhost:14557','gcs_url':'',
            'target_system_id':1,'target_component_id':1,'fcu_protocol':'v2.0',
            'plugin_denylist':['odometry','landing_target','safety_area','image_pub','vibration','distance_sensor','rangefinder','wheel_odometry']}]),
        TimerAction(period=15.,actions=[Node(package='fast_lio',executable='fastlio_mapping',output='screen',
            parameters=[os.path.join(share,'config/mid360_sim.yaml'),{'use_sim_time':True}])]),
        TimerAction(period=18.,actions=[Node(package='astra_px4_sim',executable='lio_vision_bridge.py',output='screen',parameters=[{'use_sim_time':True}]),
            Node(package='astra_wall_control',executable='configure_estimator.py',output='screen')]),
        TimerAction(period=35.,actions=[Node(package='astra_wall_control',executable='wall_control.py',output='screen',parameters=[{
            'use_sim_time':True,
            'control_mode':LaunchConfiguration('control_mode'),
            'hold_body_motion':ParameterValue(LaunchConfiguration('hold_body_motion'),value_type=bool),
            'body_motion_period':ParameterValue(LaunchConfiguration('body_motion_period'),value_type=float),
            'body_motion_bounds':ParameterValue(LaunchConfiguration('body_motion_bounds'),value_type=List[float]),
            'body_motion_amplitude':ParameterValue(LaunchConfiguration('body_motion_amplitude'),value_type=List[float]),
            'takeoff_height':ParameterValue(LaunchConfiguration('takeoff_height'),value_type=float),
            'corridor_file':LaunchConfiguration('corridor_file'),
            'interactive_goal':ParameterValue(LaunchConfiguration('interactive_goal'),value_type=bool),
            'goal_height':ParameterValue(LaunchConfiguration('goal_height'),value_type=float),
            'auto_arm':ParameterValue(LaunchConfiguration('auto_arm'),value_type=bool)}])]),
    ])
