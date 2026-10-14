#!/usr/bin/env python3
"""Read-only SITL evaluator. Simulator truth never feeds the control graph."""
import argparse
import json
import sys
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ContactsState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from astra_wall_control.controller import ArmModel
from astra_wall_control.geometry import NAMES
from wholebody_catch.mission import touches_target


class Evaluator(Node):
    def __init__(self):
        super().__init__('catch_evaluator')
        share = Path(get_package_share_directory('astra_wall_control'))
        self.model = ArmModel(share/'models/my/arm.urdf')
        self.object = self.body = self.q = self.status = None
        self.received = {}
        self.stamps = {}
        self.baseline = None
        self.started_early = False
        self.since = None
        self.last_sim = None
        self.passed = False
        self.failed = ''
        self.samples = []
        self.contacts = [(False, -1.), (False, -1.)]
        self.last_touch = [-1., -1.]
        for index, side in enumerate(('left', 'right')):
            self.create_subscription(ContactsState, '/my/arm/gripper_'+side+'_contact',
                                     lambda msg, i=index: self.contact(msg, i), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wholebody_catch/eval/object',
                                 lambda msg: self.odom(msg, 'object'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wall_eval/body',
                                 lambda msg: self.odom(msg, 'body'), qos_profile_sensor_data)
        self.create_subscription(JointState, '/my/arm/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(String, '/wholebody_catch/status', self.mission, 10)

    def odom(self, msg, name):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        value = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        if np.isfinite(value).all() and np.linalg.norm(value[3:]) > .5:
            setattr(self, name, value)
            self.received[name] = time.monotonic()
            self.stamps[name] = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def contact(self, msg, index):
        self.contacts[index] = (touches_target(msg.states, 'catch_target'),
                                msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        if self.contacts[index][0]:
            self.last_touch[index] = self.contacts[index][1]

    def joints(self, msg):
        if len(msg.name) != 7 or set(msg.name) != set(NAMES) or len(msg.position) != 7:
            return
        q = np.array([msg.position[msg.name.index(n)] for n in NAMES])
        if np.isfinite(q).all():
            self.q = q
            self.received['q'] = time.monotonic()
            self.stamps['q'] = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def mission(self, msg):
        self.status = json.loads(msg.data)
        self.received['status'] = time.monotonic()
        self.stamps['status'] = self.status['stamp']
        if self.status['phase'] in ('WAIT_FLIGHT', 'SEARCH', 'PREGRASP'):
            self.started_early = True
        if self.status['phase'] == 'ABORT':
            self.failed = 'Mission aborted: '+self.status['reason']

    def check(self):
        if len(self.received) != 4 or any(time.monotonic()-t > .8 for t in self.received.values()):
            self.since = None
            return
        now = self.stamps['object']
        if max(self.stamps.values())-min(self.stamps.values()) > .3:
            self.since = None
            return
        if self.last_sim is not None and now < self.last_sim:
            self.failed = 'Simulation clock reset'
            return
        if now == self.last_sim:
            return
        self.last_sim = now
        if self.baseline is None:
            if not self.started_early:
                self.failed = 'Start evaluator before the gripper closes; no pre-grasp baseline'
                return
            self.baseline = self.object[:3].copy()
        tip = self.model.evaluate(self.body[:3], Rotation.from_quat(self.body[3:]).as_matrix(), self.q)[0]
        lift = float(self.object[2]-self.baseline[2])
        error = float(np.linalg.norm(self.object[:3]-tip))
        bilateral = all(abs(now-stamp) < .2 and -.04 <= now-touch < .08
                        for (_, stamp), touch in zip(self.contacts, self.last_touch))
        good = (self.status['phase'] == 'SUCCESS' and bilateral
                and lift >= .12 and error < .04)
        self.since = (now if self.since is None else self.since) if good else None
        self.samples.append(dict(t=now, phase=self.status['phase'], object=self.object[:3].tolist(),
                                 tip=tip.tolist(), lift=lift, object_tip_error=error,
                                 bilateral_contact=bilateral))
        self.passed = self.since is not None and now-self.since >= 2.


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=240., help='Wall-clock timeout; increase for slow simulation')
    parser.add_argument('--output', default='/tmp/wholebody_catch_validation.json')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    rclpy.init()
    node = Evaluator()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-started < args.seconds and not node.passed and not node.failed:
            rclpy.spin_once(node, timeout_sec=.05)
            node.check()
    except KeyboardInterrupt:
        node.failed = 'Evaluation interrupted'
    finally:
        report = dict(passed=node.passed, reason=node.failed or ('verified physical lift' if node.passed else 'timeout'),
                      baseline=None if node.baseline is None else node.baseline.tolist(), samples=node.samples)
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2))
        print(json.dumps({key: value for key, value in report.items() if key != 'samples'}))
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if node.passed else 1)


if __name__ == '__main__':
    main()
