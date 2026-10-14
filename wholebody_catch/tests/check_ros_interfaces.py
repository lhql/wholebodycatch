#!/usr/bin/env python3
"""Source-only ROS loopback check with synthetic sensors, no PX4/Gazebo."""
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
from unittest.mock import patch

import numpy as np
import rclpy
import launch_ros.actions  # Load system launch resources before source-share patching.
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import State
from launch import LaunchContext
from launch_ros.substitutions import ExecutableInPackage
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from astra_wall_control.controller import ArmModel
from astra_wall_control.geometry import NAMES
from test_core import render_sphere


ROOT = Path(__file__).resolve().parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    # Exercise the same executable lookup as ros2 launch against temporary
    # symlinks to source scripts. Import-only checks miss absent execute bits.
    with TemporaryDirectory(prefix='catch_exec_check_') as directory:
        libexec = Path(directory)/'lib/wholebody_catch'
        libexec.mkdir(parents=True)
        with patch('launch_ros.substitutions.find_package.get_package_prefix', return_value=directory):
            for script in ('catch_vision.py', 'catch_mission.py', 'evaluate_catch.py', 'object_vision.py', 'make_catch_scene.py'):
                (libexec/script).symlink_to(ROOT/'scripts'/script)
                found = ExecutableInPackage(executable=script, package='wholebody_catch').perform(LaunchContext())
                assert Path(found).resolve() == ROOT/'scripts'/script
    rclpy.init()
    # Resolve resources directly to source trees; never source workspace install/.
    def share(package):
        return str(ROOT.parent/package)
    with patch('ament_index_python.packages.get_package_share_directory', share), \
            patch('ament_index_python.packages.get_package_prefix', lambda package: '/unused/source-check'):
        vision = load('source_catch_vision', 'scripts/catch_vision.py').CatchVision()
        mission = load('source_catch_mission', 'scripts/catch_mission.py').MissionNode()
        description = load('source_catch_launch', 'launch/catch.launch.py').generate_launch_description()
        assert len(description.entities) >= 15
        real_perception = load('source_perception_launch', 'launch/perception.launch.py').generate_launch_description()
        assert len(real_perception.entities) >= 6
    harness = Node('catch_source_test')
    executor = SingleThreadedExecutor()
    for node in (vision, mission, harness):
        executor.add_node(node)
    rgb, depth, k = render_sphere((0., 0., .5))
    bridge = CvBridge()
    rgb_msg = bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
    depth_msg = bridge.cv2_to_imgmsg(depth.astype('float32'), encoding='32FC1')
    info = CameraInfo()
    info.width, info.height, info.k = 320, 240, k.ravel().tolist()
    image_pub = harness.create_publisher(type(rgb_msg), '/my/arm/camera/image_raw', 10)
    depth_pub = harness.create_publisher(type(depth_msg), '/my/arm/camera/depth/image_raw', 10)
    info_pub = harness.create_publisher(CameraInfo, '/my/arm/camera/camera_info', 10)
    telemetry = harness.create_publisher(String, '/wall_control/telemetry', 10)
    state_pub = harness.create_publisher(State, '/mavros/state', 10)
    references = harness.create_publisher(JointState, '/wholebody_catch/arm_reference', 10)
    detections, commands, statuses = [], [], []
    harness.create_subscription(PointStamped, '/wholebody_catch/target', detections.append, 10)
    harness.create_subscription(JointState, '/my/arm/position_command', commands.append, 10)
    harness.create_subscription(String, '/wholebody_catch/status', lambda m: statuses.append(json.loads(m.data)), 10)
    q = [0., -.05, .1, .05, 0., .018, .018]
    model = ArmModel(ROOT.parent/'astra_wall_control/models/my/arm.urdf')
    tip = model.evaluate(np.array([0., 0., 1.6]), np.eye(3), np.array(q))[0]
    expected = model.frames['arm_camera_optical_frame'] @ np.array([0., 0., .5, 1.])
    try:
        deadline = time.monotonic()+4.
        next_sample = 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_sample:
                stamp = harness.get_clock().now().to_msg()
                t = stamp.sec+stamp.nanosec*1e-9
                row = dict(t=t, body=[0., 0., 1.6], body_orientation=[0., 0., 0., 1.], q=q,
                           tip=tip.tolist(), tool_angle=0., phase='HOLD', armed=True, mode='OFFBOARD')
                telemetry.publish(String(data=json.dumps(row)))
                state_pub.publish(State(connected=True, armed=True, mode='OFFBOARD'))
                for msg, publisher in ((rgb_msg, image_pub), (depth_msg, depth_pub), (info, info_pub)):
                    msg.header.stamp, msg.header.frame_id = stamp, 'arm_camera_optical_frame'
                    publisher.publish(msg)
                ref = JointState()
                ref.header.stamp, ref.name, ref.position, ref.effort = stamp, NAMES, q, [0.]*7
                references.publish(ref)
                next_sample = time.monotonic()+.05
            executor.spin_once(timeout_sec=.005)
        assert len(detections) > 10, 'No synchronized vision output'
        actual = detections[-1].point
        np.testing.assert_allclose([actual.x, actual.y, actual.z], expected[:3], atol=.001)
        assert commands, 'No routed arm commands'
        np.testing.assert_allclose(commands[-1].position[:5], q[:5])
        assert statuses[-1]['phase'] == 'PREGRASP', statuses[-1]
        # The base controller stops telemetry on pilot mode changes. The
        # mission must consult live MAVROS state instead of that stale row.
        state_pub.publish(State(connected=True, armed=True, mode='POSCTL'))
        deadline = time.monotonic()+.25
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert mission.machine.phase == 'ABORT'
        assert mission.pending is None, 'Pilot mode change must not request AUTO.LAND'
        abort = harness.create_client(Trigger, '/wholebody_catch/abort')
        assert abort.wait_for_service(timeout_sec=1.)
        future = abort.call_async(Trigger.Request())
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline and not future.done():
            executor.spin_once(timeout_sec=.01)
        assert future.done() and future.result().success
        assert mission.machine.phase == 'ABORT'
        from wholebody_catch.mission import CatchMission
        mission.machine = CatchMission()
        mission.flight_state = State(connected=True, armed=True, mode='POSCTL')
        mission.state_wall = time.monotonic()
        mission.row_wall = time.monotonic()-2.
        mission.tick()
        assert mission.machine.reason == 'flight telemetry stale during takeoff'
        mission.machine = CatchMission()
        mission.flight_state = None
        mission.started_wall = time.monotonic()-mission.startup_timeout-1.
        mission.tick()
        assert mission.machine.reason.startswith('startup timeout')
        # Closing against an object must not integrate the position reference
        # through the measured fingers; stale encoders cannot drive closure.
        captured = []
        original_pub = mission.arm_pub
        class Capture:
            def publish(self, msg):
                captured.append(msg)
        mission.arm_pub = Capture()
        ref.header.stamp = mission.get_clock().now().to_msg()
        now = ref.header.stamp.sec+ref.header.stamp.nanosec*1e-9
        mission.machine.gripper = -.018
        mission.finger_reference = np.array([-.018, -.018])
        mission.joint_feedback = (now, np.array([.002, -.001]))
        mission.last_tick_wall = time.monotonic()
        mission.arm_reference(ref)
        np.testing.assert_allclose(captured[-1].position[-2:], [.0012, -.0018])
        mission.joint_feedback = (now-1., np.array([.002, -.001]))
        mission.arm_reference(ref)
        assert len(captured) == 1
        mission.arm_pub = original_pub
        original_status = mission.status
        captured.clear()
        mission.status = Capture()
        mission.contacts = [(False, now), (False, now)]
        mission.last_touch = [now-.04, now-.04]
        mission.last_status = -1e9
        mission.tick()
        assert json.loads(captured[-1].data)['bilateral_contact']
        mission.last_touch = [now-.2, now-.2]
        mission.last_status = -1e9
        mission.tick()
        assert not json.loads(captured[-1].data)['bilateral_contact']
        mission.status = original_status
        print('PASS: ROS executable discovery through symlinks, source launch, RGB-D synchronization, timestamped map projection, arm routing, mission and abort service')
    finally:
        executor.shutdown()
        for node in (vision, mission, harness):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
