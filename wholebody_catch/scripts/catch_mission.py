#!/usr/bin/env python3
"""Mission and sole arm-command output; the existing WBC supplies five axes."""
import copy
import json
import time

import numpy as np
import rclpy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from astra_wall_control.geometry import NAMES
from wholebody_catch.mission import CatchMission, Settings, touches_target


def seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class MissionNode(Node):
    def __init__(self):
        super().__init__('catch_mission')
        for key, value in vars(Settings()).items():
            self.declare_parameter(key, value)
        self.declare_parameter('target_model', 'catch_target')
        self.declare_parameter('contact_source', 'gazebo')
        self.declare_parameter('workspace_min', [-.7, -1.2, .9])
        self.declare_parameter('workspace_max', [2.8, 1.2, 2.])
        self.declare_parameter('startup_timeout', 180.)
        self.startup_timeout = float(self.get_parameter('startup_timeout').value)
        if not np.isfinite(self.startup_timeout) or self.startup_timeout < 30.:
            raise ValueError('startup_timeout must be at least 30 wall-clock seconds')
        self.started_wall = time.monotonic()
        self.machine = CatchMission(Settings(**{key: self.get_parameter(key).value for key in vars(Settings())}))
        self.target_model = self.get_parameter('target_model').value
        self.row = None
        self.row_wall = -1e9
        self.flight_state = None
        self.state_wall = -1e9
        self.last_tick_wall = time.monotonic()
        self.contacts = [(False, -1e9), (False, -1e9)]
        self.last_touch = [-1e9, -1e9]
        self.joint_feedback = None
        self.last_goal = None
        self.last_reference = None
        self.finger_reference = None
        self.last_status = -1e9
        self.last_landing = -1e9
        self.pending = None
        self.previous_tip = None
        self.speed = 0.
        self.previous_phase = self.machine.phase
        self.arm_pub = self.create_publisher(JointState, '/my/arm/position_command', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/wholebody_catch/ee_goal', 10)
        self.status = self.create_publisher(String, '/wholebody_catch/status', 10)
        self.mode = self.create_client(SetMode, '/mavros/set_mode')
        self.create_subscription(String, '/wall_control/telemetry', self.telemetry, 10)
        self.create_subscription(State, '/mavros/state', self.flight_status, 10)
        self.create_subscription(PointStamped, '/wholebody_catch/target', self.target, 10)
        self.create_subscription(JointState, '/wholebody_catch/arm_reference', self.arm_reference, 1)
        self.create_subscription(JointState, '/my/arm/joint_states', self.joints, qos_profile_sensor_data)
        contact_source = self.get_parameter('contact_source').value
        for index, side in enumerate(('left', 'right')):
            if contact_source == 'gazebo':
                self.create_subscription(ContactsState, '/my/arm/gripper_'+side+'_contact',
                                         lambda msg, i=index: self.contact(msg, i), qos_profile_sensor_data)
            elif contact_source == 'tactile':
                self.create_subscription(Bool, '/wholebody_catch/tactile/'+side,
                                         lambda msg, i=index: self.tactile(msg, i), qos_profile_sensor_data)
            else:
                raise ValueError('contact_source must be gazebo or tactile')
        self.create_service(Trigger, '/wholebody_catch/abort', self.abort_service)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def telemetry(self, msg):
        try:
            row = json.loads(msg.data)
            values = np.r_[row['t'], row['tip'], row['q'], row['tool_angle']]
            if values.shape != (12,) or not np.isfinite(values).all():
                raise ValueError('Invalid telemetry')
            if self.previous_tip is not None:
                dt = row['t']-self.previous_tip[0]
                if 0 < dt < .6:
                    self.speed = float(np.linalg.norm(np.array(row['tip'])-self.previous_tip[1])/dt)
            self.previous_tip = (row['t'], np.array(row['tip']))
            self.row, self.row_wall = row, time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.machine.abort('invalid flight telemetry')

    def target(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        stamp = seconds(msg.header.stamp)
        if msg.header.frame_id == 'map' and 0 <= now-stamp < .6:
            point = np.array([msg.point.x, msg.point.y, msg.point.z])
            if (np.all(point >= self.get_parameter('workspace_min').value)
                    and np.all(point <= self.get_parameter('workspace_max').value)):
                self.machine.observe(stamp, point)

    def flight_status(self, msg):
        self.flight_state, self.state_wall = msg, time.monotonic()

    def contact(self, msg, index):
        self.contacts[index] = (touches_target(msg.states, self.target_model), seconds(msg.header.stamp))
        if self.contacts[index][0]:
            self.last_touch[index] = self.contacts[index][1]

    def tactile(self, msg, index):
        stamp = self.get_clock().now().nanoseconds*1e-9
        self.contacts[index] = (bool(msg.data), stamp)
        if msg.data:
            self.last_touch[index] = stamp

    def joints(self, msg):
        if (len(msg.name) == 7 and set(msg.name) == set(NAMES) and len(msg.position) == 7
                and np.isfinite(msg.position).all()):
            self.joint_feedback = (seconds(msg.header.stamp),
                                   np.array([msg.position[msg.name.index(n)] for n in NAMES[-2:]]))

    def arm_reference(self, msg):
        now = self.get_clock().now().nanoseconds*1e-9
        stamp = seconds(msg.header.stamp)
        # Never synthesize a live arm command from an old/missing WBC command.
        if (time.monotonic()-self.last_tick_wall > .25 or not 0 <= now-stamp < .15
                or len(msg.name) != 7 or set(msg.name) != set(NAMES)
                or len(msg.position) != 7 or len(msg.effort) != 7
                or not np.isfinite(msg.position).all() or not np.isfinite(msg.effort).all()):
            return
        dt = .02 if self.last_reference is None else stamp-self.last_reference
        if dt < 0:
            self.machine.abort('arm reference clock reset')
            return
        self.last_reference = stamp
        out = copy.deepcopy(msg)
        indices = [msg.name.index(name) for name in NAMES[-2:]]
        if self.finger_reference is None:
            self.finger_reference = np.array([msg.position[i] for i in indices])
        limit = .012*min(dt, .05)
        self.finger_reference += np.clip(self.machine.gripper-self.finger_reference, -limit, limit)
        if self.machine.gripper < 0.:
            if self.joint_feedback is None or not 0 <= now-self.joint_feedback[0] < .15:
                return
            # Limit PD spring compression against measured fingers. Avoid
            # integrating the full closed reference through a grasped object.
            self.finger_reference = np.maximum(self.finger_reference, self.joint_feedback[1]-.0008)
        for i, value in zip(indices, self.finger_reference):
            out.position[i] = float(value)
        self.arm_pub.publish(out)

    def abort_service(self, request, response):
        self.machine.abort('operator abort')
        response.success, response.message = True, 'Grasp aborted; AUTO.LAND requested if still in OFFBOARD.'
        return response

    def landing(self, wall):
        if self.pending is not None and not self.pending.done() and wall-self.last_landing > 3.:
            self.pending.cancel()
            self.pending = None
            self.get_logger().warning('AUTO.LAND service timed out; will retry')
        if self.pending is not None and self.pending.done():
            try:
                result = self.pending.result()
                if not result.mode_sent:
                    self.get_logger().warning('AUTO.LAND request not accepted; will retry')
            except Exception as exc:
                self.get_logger().warning(str(exc))
            self.pending = None
        if (self.flight_state is not None and wall-self.state_wall < 3.
                and self.flight_state.armed and self.flight_state.mode == 'OFFBOARD'
                and wall-self.last_landing > 2. and self.pending is None and self.mode.service_is_ready()):
            self.pending = self.mode.call_async(SetMode.Request(custom_mode='AUTO.LAND'))
            self.last_landing = wall

    def tick(self):
        wall = time.monotonic()
        now = self.get_clock().now().nanoseconds*1e-9
        self.last_tick_wall = wall
        row = self.row
        fresh = row is not None and wall-self.row_wall < .8 and 0 <= now-row['t'] < .6
        ready = (fresh and row.get('phase') in ('APPROACH', 'HOLD')
                 and self.flight_state is not None and wall-self.state_wall < 3.
                 and self.flight_state.armed and self.flight_state.mode == 'OFFBOARD'
                 and row.get('armed') and row.get('mode') == 'OFFBOARD')
        bilateral = all(-.03 <= now-stamp < .2 and -.03 <= now-touch < .08
                        for (_, stamp), touch in zip(self.contacts, self.last_touch))
        if self.machine.phase == 'WAIT_FLIGHT':
            if wall-self.started_wall > self.startup_timeout:
                self.machine.abort('startup timeout: flight controller never became ready')
            elif (self.flight_state is not None and self.flight_state.armed
                  and wall-self.state_wall < 3. and row is not None and not fresh):
                self.machine.abort('flight telemetry stale during takeoff')
        if self.machine.phase != 'WAIT_FLIGHT' and not fresh:
            self.machine.abort('flight telemetry stale')
        if row and row.get('phase') in ('ABORT', 'STALE', 'CLOCK_RESET', 'TAKEOFF_TIMEOUT', 'SOLVER_FAULT'):
            self.machine.abort('flight controller: '+row['phase'])
        self.machine.update(now, ready, row['tip'] if row else [0., 0., 0.], self.speed,
                            row['tool_angle'] if row else 0.,
                            fresh and min(row['q'][-2:]) >= .012, bilateral)
        if self.machine.phase == 'ABORT':
            self.landing(wall)
        elif ready and self.machine.goal is not None:
            goal = self.machine.goal
            if self.last_goal is None or np.linalg.norm(goal-self.last_goal) > .001:
                msg = PoseStamped()
                msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, goal)
                msg.pose.orientation.w = 1.
                self.goal_pub.publish(msg)
                self.last_goal = goal.copy()
        if wall-self.last_status > .2:
            self.status.publish(String(data=json.dumps(dict(
                stamp=now, phase=self.machine.phase, reason=self.machine.reason,
                target=None if self.machine.target is None else self.machine.target.tolist(),
                goal=None if self.machine.goal is None else self.machine.goal.tolist(),
                bilateral_contact=bilateral, gripper=self.machine.gripper,
                recoveries=self.machine.recoveries,
                planned_blind=self.machine.blind, verification=self.machine.verification,
                approach_endpoint=None if self.machine.plan is None else self.machine.plan.end.tolist(),
                approach_remaining_time=None if self.machine.plan is None else self.machine.plan.remaining_time,
                left_contact=bool(self.contacts[0][0] and 0 <= now-self.contacts[0][1] < .2),
                right_contact=bool(self.contacts[1][0] and 0 <= now-self.contacts[1][1] < .2),
                tip_speed=self.speed,
                tracking_error=None if row is None or self.machine.goal is None else
                    float(np.linalg.norm(np.asarray(row['tip'])-self.machine.goal)),
                target_error=None if row is None or self.machine.target is None else
                    float(np.linalg.norm(np.asarray(row['tip'])-self.machine.target)),
                vision_age=None if self.machine.last_observation is None else
                    now-self.machine.last_observation[0],
                flight_phase=None if row is None else row.get('phase')))))
            self.last_status = wall
        if self.previous_phase != self.machine.phase:
            self.get_logger().info(self.machine.phase+': '+self.machine.reason)
            self.previous_phase = self.machine.phase


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
