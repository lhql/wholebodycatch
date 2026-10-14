"""Isolated Gazebo/PX4 + FAST-LIO2 + wrist-vision grasping experiment."""
import os

import numpy as np
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from astra_wall_control.controller import ArmModel


def generate_launch_description():
    share = get_package_share_directory('wholebody_catch')
    base = get_package_share_directory('astra_wall_control')
    runner = os.path.join(get_package_prefix('astra_wall_control'), 'lib/astra_wall_control/run_sim.sh')
    # Initial goal is a hover above the spawn point, unrelated to object position.
    model = ArmModel(os.path.join(base, 'models/my/arm.urdf'))
    offset = model.evaluate(np.zeros(3), np.eye(3), np.array([0., -.05, .1, .05, 0., .01, .01]))[0]
    initial_goal = (offset+np.array([0., 0., 1.6])).tolist()
    common = {'use_sim_time': True}
    sim = ExecuteProcess(cmd=[runner, 'my', LaunchConfiguration('world'), LaunchConfiguration('gui'),
                              base, LaunchConfiguration('lidar_downsample')], output='screen')
    vision = Node(package='wholebody_catch', executable='catch_vision.py', output='screen',
                  parameters=[os.path.join(share, 'config/catch.yaml'), common],
                  condition=IfCondition(PythonExpression(["'", LaunchConfiguration('perception'), "' == 'sphere'"])))
    objects = Node(package='wholebody_catch', executable='object_vision.py', output='screen',
                   parameters=[os.path.join(share, 'config/catch.yaml'), common, dict(python_executable=LaunchConfiguration('inference_python'),
                       sam2_repository=os.path.join(share, 'vendor/sam2'),
                       model_weights=os.path.join(share, 'weights/sam2.1_hiera_tiny.pt'),
                       extra_python=os.path.join(share, 'vendor/python311'))],
                   condition=IfCondition(PythonExpression(["'", LaunchConfiguration('perception'), "' == 'sam2'"])))
    mission = Node(package='wholebody_catch', executable='catch_mission.py', output='screen',
                   parameters=[os.path.join(share, 'config/catch.yaml'), common])
    controller = Node(package='astra_wall_control', executable='wall_control.py', output='screen',
                      parameters=[dict(common, target=initial_goal, interactive_goal=True,
                                       control_mode='wholebody', hold_body_motion=False,
                                       wholebody_velocity_damping=.7,
                                       measured_motion_compensation=True,
                                       auto_arm=ParameterValue(LaunchConfiguration('auto_arm'), value_type=bool))],
                      remappings=[('/my/arm/position_command', '/wholebody_catch/arm_reference'),
                                  ('/ee_goal', '/wholebody_catch/ee_goal'),
                                  ('/goal_pose', '/wholebody_catch/unused_goal_pose'),
                                  ('/wholebody/body_goal', '/wholebody_catch/unused_body_goal'),
                                  ('/wholebody/motion_enabled', '/wholebody_catch/unused_motion_enabled')])
    lio = Node(package='fast_lio', executable='fastlio_mapping', output='screen',
               parameters=[os.path.join(base, 'config/mid360_sim.yaml'), common])
    actions = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('auto_arm', default_value='true'),
        DeclareLaunchArgument('perception', default_value='sam2', choices=['sam2', 'sphere']),
        DeclareLaunchArgument('inference_python', default_value=os.environ.get('CATCH_PYTHON', os.path.expanduser('~/miniconda3/envs/torch/bin/python'))),
        DeclareLaunchArgument('evaluate', default_value='false'),
        DeclareLaunchArgument('evaluation_output', default_value='/tmp/wholebody_catch_validation.json'),
        DeclareLaunchArgument('lidar_downsample', default_value='4'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds/catch.world')),
        DeclareLaunchArgument('rootfs', default_value='/tmp/wholebody_catch_px4'),
        SetEnvironmentVariable('ASTRA_WALL_ROOTFS', LaunchConfiguration('rootfs')),
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),
    ]
    # A dead command router must not leave an unattended controller running.
    # This is a simulation-only launch: shutdown also stops the simulator/PX4.
    for process in (sim, vision, objects, mission, controller, lio):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=[EmitEvent(event=Shutdown(reason='catch stack process exited'))])))
    actions.extend([
        sim, vision, objects, mission,
        Node(package='wholebody_catch', executable='evaluate_catch.py', output='screen',
             arguments=['--seconds', '300', '--output', LaunchConfiguration('evaluation_output')],
             condition=IfCondition(LaunchConfiguration('evaluate'))),
        Node(package='mavros', executable='mavros_node', namespace='mavros', name='mavros',
             output='screen', parameters=[dict(common, fcu_url='udp://:14540@localhost:14557', gcs_url='',
                 target_system_id=1, target_component_id=1, fcu_protocol='v2.0',
                 plugin_denylist=['odometry', 'landing_target', 'safety_area', 'image_pub',
                                 'vibration', 'distance_sensor', 'rangefinder', 'wheel_odometry'])]),
        TimerAction(period=15., actions=[lio]),
        TimerAction(period=18., actions=[
            Node(package='astra_px4_sim', executable='lio_vision_bridge.py', output='screen', parameters=[common]),
            Node(package='astra_wall_control', executable='configure_estimator.py', output='screen')]),
        TimerAction(period=35., actions=[controller]),
        Node(package='astra_wall_control', executable='air_goal.py', output='screen',
             parameters=[dict(common, enable_goal_input=False, control_mode='wholebody',
                              scene_file=LaunchConfiguration('world'))],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2', executable='rviz2', output='screen', parameters=[common],
             arguments=['-d', os.path.join(share, 'config/catch.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
    return LaunchDescription(actions)
