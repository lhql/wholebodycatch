#!/usr/bin/env python3
"""Synchronized wrist RGB-D -> target center in the controller's map frame."""
from collections import deque
import json
from pathlib import Path

import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from astra_wall_control.geometry import Kinematics
from wholebody_catch.perception import StateHistory, detect_sphere, optical_to_map


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class CatchVision(Node):
    def __init__(self):
        super().__init__('catch_vision')
        defaults = dict(rgb_topic='/my/arm/camera/image_raw',
                        depth_topic='/my/arm/camera/depth/image_raw',
                        info_topic='/my/arm/camera/camera_info', target_radius=.02)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.bridge = CvBridge()
        self.history = StateHistory()
        self.pending = deque(maxlen=20)
        self.last_image = -1.
        self.last_report = -1.
        self.radius = float(self.get_parameter('target_radius').value)
        if not .012 <= self.radius <= .028:
            raise ValueError('target_radius must fit the gripper: [0.012, 0.028] m')
        share = Path(get_package_share_directory('astra_wall_control'))
        self.kinematics = Kinematics(share/'models/my/arm.urdf')
        self.pub = self.create_publisher(PointStamped, '/wholebody_catch/target', 10)
        self.status = self.create_publisher(String, '/wholebody_catch/vision_status', 10)
        self.create_subscription(String, '/wall_control/telemetry', self.telemetry, 10)
        self.rgb = message_filters.Subscriber(self, Image, self.get_parameter('rgb_topic').value,
                                              qos_profile=qos_profile_sensor_data)
        self.depth = message_filters.Subscriber(self, Image, self.get_parameter('depth_topic').value,
                                                qos_profile=qos_profile_sensor_data)
        self.info = message_filters.Subscriber(self, CameraInfo, self.get_parameter('info_topic').value,
                                               qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb, self.depth, self.info], 20, .025)
        self.sync.registerCallback(self.images)
        self.create_timer(.05, self.process)

    def telemetry(self, msg):
        try:
            row = json.loads(msg.data)
            if self.history.rows and row['t'] < self.history.rows[-1]['t']:
                self.pending.clear()
                self.last_image = -1.
            self.history.add(row)
        except (ValueError, KeyError, TypeError):
            self.history.rows.clear()

    def images(self, rgb, depth, info):
        stamp = seconds(rgb.header.stamp)
        now = self.get_clock().now().nanoseconds*1e-9
        if (stamp <= self.last_image or not 0 <= now-stamp < .6
                or {rgb.header.frame_id, depth.header.frame_id, info.header.frame_id} != {'arm_camera_optical_frame'}
                or (rgb.width, rgb.height) != (depth.width, depth.height)
                or (rgb.width, rgb.height) != (info.width, info.height)):
            return
        self.last_image = stamp
        self.pending.append((stamp, rgb, depth, info))

    def process(self):
        now = self.get_clock().now().nanoseconds*1e-9
        selected = None
        while self.pending:
            sample = self.pending[0]
            if now-sample[0] > .6 or now < sample[0]:
                self.pending.popleft()
                continue
            state = self.history.sample(sample[0])
            if state is None:
                if self.history.rows and sample[0] < self.history.rows[-1]['t']:
                    self.pending.popleft()
                    continue
                break  # Wait for the next body sample to bracket exposure time.
            selected = (self.pending.popleft(), state)
        if selected is None:
            if now-self.last_report > 2.:
                self.status.publish(String(data=json.dumps(dict(state='WAIT_SYNCHRONIZED_RGBD_AND_POSE', stamp=now))))
                self.last_report = now
            return
        (stamp, rgb, depth, info), state = selected
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            d = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough').astype(float)
            if depth.encoding == '16UC1':
                d *= .001
            elif depth.encoding != '32FC1':
                raise ValueError('Depth must be 32FC1 metres or 16UC1 millimetres')
            result = detect_sphere(bgr, d, info.k, self.radius)
            report = dict(stamp=stamp, state='NO_UNAMBIGUOUS_TARGET')
            if result is not None:
                optical, diagnostics = result
                target = optical_to_map(optical, state, self.kinematics)
                msg = PointStamped()
                msg.header = rgb.header
                msg.header.frame_id = 'map'
                msg.point.x, msg.point.y, msg.point.z = map(float, target)
                self.pub.publish(msg)
                report.update(state='DETECTED', point=target.tolist(), **diagnostics)
            self.status.publish(String(data=json.dumps(report)))
            self.last_report = now
        except (ValueError, TypeError, RuntimeError) as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=3.)


def main():
    rclpy.init()
    node = CatchVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
