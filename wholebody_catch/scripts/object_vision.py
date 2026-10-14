#!/usr/bin/env python3
"""Class-agnostic SAM 2 + registered depth; exposure-time transforms only."""
import base64
from collections import deque
import json
import os
from pathlib import Path
import queue
import select
import subprocess
import threading
import zlib

import cv2
import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from wholebody_catch.perception import StateHistory
from wholebody_catch.tracking import TargetTracker


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class ObjectVision(Node):
    def __init__(self):
        super().__init__('object_vision')
        defaults = dict(rgb_topic='/my/arm/camera/image_raw', depth_topic='/my/arm/camera/depth/image_raw',
                        info_topic='/my/arm/camera/camera_info', camera_frame='arm_camera_optical_frame',
                        world_frame='map', pose_source='telemetry', urdf_path='',
                        python_executable='', sam2_repository='', model_weights='', extra_python='',
                        workspace_min=[.6, -1., .9], workspace_max=[2.5, 1., 1.9],
                        max_width=.06, surface_insertion=.01, maximum_result_age=.6)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key in ('python_executable', 'sam2_repository', 'model_weights'):
            if not Path(self.cfg[key]).exists() or not self.cfg[key]:
                raise ValueError('Configure an existing '+key+'; see README setup instructions')
        self.history = StateHistory()
        self.kinematics = None
        if self.cfg['pose_source'] == 'telemetry':
            from astra_wall_control.geometry import Kinematics
            path = self.cfg['urdf_path'] or str(Path(get_package_share_directory('astra_wall_control'))/'models/my/arm.urdf')
            self.kinematics = Kinematics(path)
            self.create_subscription(String, '/wall_control/telemetry', self.telemetry, 10)
        elif self.cfg['pose_source'] != 'tf':
            raise ValueError('pose_source must be telemetry or tf')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.bridge = CvBridge()
        self.tracker = TargetTracker()
        self.images_pending = deque(maxlen=30)
        self.last_received = -1.
        self.last_image = -1.
        self.last_report = -1.
        self.reset_pending = False
        self.ready = False
        self.busy = threading.Event()
        self.requests = queue.Queue(maxsize=1)
        self.results = queue.Queue()
        self.process_handle = None
        self.pub = self.create_publisher(PointStamped, '/wholebody_catch/target', 10)
        self.status = self.create_publisher(String, '/wholebody_catch/vision_status', 10)
        self.create_service(Trigger, '/wholebody_catch/reset_perception', self.reset)
        self.subs = [message_filters.Subscriber(self, kind, self.cfg[key], qos_profile=qos_profile_sensor_data)
                     for kind, key in ((Image, 'rgb_topic'), (Image, 'depth_topic'), (CameraInfo, 'info_topic'))]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, 20, .025)
        self.sync.registerCallback(self.images)
        self.thread = threading.Thread(target=self.worker, daemon=True)
        self.thread.start()
        self.create_timer(.03, self.tick)

    def reset(self, request, response):
        self.tracker = TargetTracker()
        self.reset_pending = True
        response.success, response.message = True, 'New object search requested'
        return response

    def telemetry(self, msg):
        try:
            row = json.loads(msg.data)
            if self.history.rows and row['t'] < self.history.rows[-1]['t']:
                self.tracker = TargetTracker(); self.reset_pending = True; self.last_image = -1.
                self.last_received = -1.
                self.images_pending.clear()
            self.history.add(row)
        except (ValueError, KeyError, TypeError):
            self.history.rows.clear()

    def images(self, rgb, depth, info):
        stamp = seconds(rgb.header.stamp)
        if (stamp <= self.last_received or {rgb.header.frame_id, depth.header.frame_id, info.header.frame_id} != {self.cfg['camera_frame']}
                or (rgb.width, rgb.height) != (depth.width, depth.height)
                or (rgb.width, rgb.height) != (info.width, info.height)):
            return
        self.last_received = stamp
        self.images_pending.append((rgb, depth, info))

    def transform(self, stamp):
        if self.kinematics is not None:
            state = self.history.sample(seconds(stamp))
            if state is None:
                return None
            p, R, q = state
            B = np.eye(4); B[:3, :3] = R; B[:3, 3] = p
            return B@self.kinematics.frames(q)[self.cfg['camera_frame']]
        try:
            msg = self.tf.lookup_transform(self.cfg['world_frame'], self.cfg['camera_frame'], rclpy.time.Time.from_msg(stamp))
            q, p = msg.transform.rotation, msg.transform.translation
            T = np.eye(4); T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [p.x, p.y, p.z]
            return T
        except Exception:
            return None

    def worker(self):
        try:
            env = os.environ.copy()
            import wholebody_catch
            env['PYTHONPATH'] = os.pathsep.join([self.cfg['extra_python'], str(Path(wholebody_catch.__file__).resolve().parents[1])])
            script = Path(get_package_share_directory('wholebody_catch'))/'scripts/segment_worker.py'
            proc = subprocess.Popen([self.cfg['python_executable'], str(script), '--vendor', self.cfg['sam2_repository'],
                                     '--weights', self.cfg['model_weights']], stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, text=True, bufsize=1, env=env)
            self.process_handle = proc
            if not select.select([proc.stdout], [], [], 90.)[0]:
                raise RuntimeError('SAM 2 startup timeout')
            ready = json.loads(proc.stdout.readline())
            if not ready.get('ready'):
                raise RuntimeError('SAM 2 did not become ready')
            self.results.put((None, ready))
            while True:
                item = self.requests.get()
                if item is None:
                    break
                stamp, request = item
                proc.stdin.write(json.dumps(request)+'\n'); proc.stdin.flush()
                if not select.select([proc.stdout], [], [], 15.)[0]:
                    raise RuntimeError('SAM 2 inference timeout')
                self.results.put((stamp, json.loads(proc.stdout.readline())))
                self.busy.clear()
        except Exception as exc:
            self.results.put((None, dict(error=str(exc))))
            self.busy.clear()

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        while not self.results.empty():
            stamp, result = self.results.get_nowait()
            if result.get('ready'):
                self.ready = True
                self.get_logger().info('SAM 2 ready on '+result['device'])
                continue
            if 'error' in result:
                if stamp is None:
                    self.ready = False
                self.get_logger().error(result['error'])
                continue
            candidate = result.get('candidate')
            age = now-seconds(stamp)
            accepted = (candidate is not None and 0 <= age < self.cfg['maximum_result_age']
                        and self.tracker.observe(seconds(stamp), candidate['point']))
            if accepted:
                msg = PointStamped(); msg.header.stamp = stamp; msg.header.frame_id = self.cfg['world_frame']
                msg.point.x, msg.point.y, msg.point.z = map(float, self.tracker.x[:3])
                self.pub.publish(msg)
            result.update(state='DETECTED' if accepted else 'NO_FRESH_ASSOCIATED_TARGET', stamp=seconds(stamp), age=age,
                          prediction_only=False, backend='sam2_depth')
            self.status.publish(String(data=json.dumps(result)))
            self.last_report = now
        selected = None
        if self.ready and not self.busy.is_set():
            while self.images_pending:
                rgb, depth, info = self.images_pending[0]
                stamp = seconds(rgb.header.stamp)
                if now-stamp > .4 or stamp <= self.last_image:
                    self.images_pending.popleft()
                    continue
                T = self.transform(rgb.header.stamp)
                if T is None:
                    break
                selected = (self.images_pending.popleft(), T)
        if selected is not None:
            (rgb, depth, info), T = selected
            stamp = seconds(rgb.header.stamp)
            if T is not None and self.last_image < stamp and 0 <= now-stamp < .4:
                d = self.bridge.imgmsg_to_cv2(depth, 'passthrough').astype('<f4')
                if depth.encoding == '16UC1':
                    d *= .001
                elif depth.encoding != '32FC1':
                    self.get_logger().error('Expected registered 32FC1 or 16UC1 depth')
                    return
                bgr = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
                request = dict(image=base64.b64encode(cv2.imencode('.png', bgr)[1]).decode(),
                               depth=base64.b64encode(zlib.compress(d.tobytes(), 1)).decode(),
                               K=list(info.k), transform=T.tolist(), workspace_min=self.cfg['workspace_min'],
                               workspace_max=self.cfg['workspace_max'], max_width=self.cfg['max_width'],
                               insertion=self.cfg['surface_insertion'], reset=self.reset_pending)
                self.reset_pending = False
                self.last_image = stamp
                self.busy.set()
                self.requests.put_nowait((rgb.header.stamp, request))
        if now-self.last_report > 2.:
            prediction = self.tracker.predict(now)
            self.status.publish(String(data=json.dumps(dict(state='WAIT_MODEL_OR_EXPOSURE_TRANSFORM', stamp=now,
                prediction_only=True, predicted_point=None if prediction is None else prediction[0][:3].tolist()))))
            self.last_report = now

    def close(self):
        proc = self.process_handle
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()


def main():
    rclpy.init(); node = ObjectVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
