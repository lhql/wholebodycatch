"""Real-robot perception only: registered RGB-D and calibrated exposure-time TF.

No simulator, flight controller, actuator publisher or auto-arm is started.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('wholebody_catch')
    defaults = dict(rgb_topic='/camera/color/image_raw', depth_topic='/camera/aligned_depth_to_color/image_raw',
                    info_topic='/camera/color/camera_info', camera_frame='camera_color_optical_frame',
                    world_frame='map', inference_python=os.environ.get('CATCH_PYTHON', os.path.expanduser('~/miniconda3/envs/torch/bin/python')))
    actions = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    parameters = {k: LaunchConfiguration(k) for k in defaults if k != 'inference_python'}
    parameters.update(pose_source='tf', use_sim_time=False,
                      python_executable=LaunchConfiguration('inference_python'),
                      sam2_repository=os.path.join(share, 'vendor/sam2'),
                      model_weights=os.path.join(share, 'weights/sam2.1_hiera_tiny.pt'),
                      extra_python=os.path.join(share, 'vendor/python311'))
    actions.append(Node(package='wholebody_catch', executable='object_vision.py', output='screen',
                        parameters=[os.path.join(share, 'config/catch.yaml'), parameters]))
    return LaunchDescription(actions)
