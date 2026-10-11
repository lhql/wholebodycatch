#!/usr/bin/python3
"""Simulation wall hold: measured MAVROS pose + joint encoders, no Gazebo truth."""
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, ParamSetV2
from rcl_interfaces.msg import ParameterValue
from astra_wall_control.controller import ArmModel, WholeBodyController
from astra_wall_control.geometry import NAMES
from astra_wall_control.goals import validate_goal
from astra_wall_control.corridor import SafeCorridor, whole_body_radius
from astra_wall_control.body_motion import BoundedBodyMotion


class WallControl(Node):
    def __init__(self):
        super().__init__('wall_control')
        share = Path(get_package_share_directory('astra_wall_control'))
        hover = json.loads((share/'config/flight.json').read_text())['MPC_THR_HOVER']
        for name, value in [('target', [2., 0., 1.4]), ('spawn_body', [0., 0., .499]),
                            ('interactive_goal', False), ('goal_height', 1.4),
                            ('control_mode', 'wholebody'), ('takeoff_height', 1.6),
                            ('hold_body_motion', False), ('body_motion_period', 32.),
                            ('wholebody_velocity_damping', 0.),
                            ('measured_motion_compensation', False),
                            ('body_motion_bounds', [.06, .06, .05, .20]),
                            ('body_motion_amplitude', [.025, .035, .02, .08]),
                            ('corridor_file', str(share/'config/corridor.json')),
                            ('auto_arm', True), ('hover_thrust', float(hover)), ('settle_seconds', 8.)]:
            self.declare_parameter(name, value)
        self.target = np.array(self.get_parameter('target').value)
        self.spawn = np.array(self.get_parameter('spawn_body').value)
        self.model = ArmModel(share/'models/my/arm.urdf')
        self.fk_goal_model = ArmModel(share/'models/my/arm.urdf')
        self.interactive = self.get_parameter('interactive_goal').value
        self.control_mode = self.get_parameter('control_mode').value
        self.velocity_damping = float(self.get_parameter('wholebody_velocity_damping').value)
        self.measured_motion_compensation = bool(self.get_parameter('measured_motion_compensation').value)
        if not 0. <= self.velocity_damping <= 1.5:
            raise ValueError('wholebody_velocity_damping must be within [0, 1.5] seconds')
        if self.control_mode not in ('manual', 'corridor', 'wholebody'):
            raise ValueError('control_mode must be manual, corridor or wholebody')
        self.control = WholeBodyController(self.model, wall_constraint=not self.interactive and self.control_mode == 'wholebody')
        self.home = np.array([0., -.05, .1, .05, 0., .01, .01])
        self.control.posture = self.home[:5].copy()
        tip, _, _ = self.model.evaluate(np.zeros(3), np.eye(3), self.home)
        self.tip_offset = tip.copy()
        self.body_target = self.target-tip
        self.body_center = self.body_target.copy()
        self.motion = BoundedBodyMotion(self.get_parameter('body_motion_bounds').value,
                                       self.get_parameter('body_motion_amplitude').value,
                                       self.get_parameter('body_motion_period').value)
        self.motion_enabled = bool(self.get_parameter('hold_body_motion').value)
        self.motion_started = False; self.motion_time = 0.; self.motion_paused = False
        if self.motion_enabled and self.control_mode == 'wholebody':
            self.control.posture = (self.model.lower[:5]+self.model.upper[:5])/2
        self.takeoff_height = self.body_target[2]
        self.corridor = None
        self.measured_position = None
        self.measured_velocity = np.zeros(3)
        self.manual_velocity = np.zeros(4); self.velocity_received = -1e9
        self.arm_goal = self.home.copy()
        if self.control_mode != 'wholebody':
            self.takeoff_height = float(self.get_parameter('takeoff_height').value)
            if not .9 <= self.takeoff_height <= 2.:
                raise ValueError('takeoff_height must be within [0.9, 2.0] m')
            self.body_target = np.array([self.spawn[0], self.spawn[1], self.takeoff_height])
            self.target = self.body_target+self.tip_offset
            if self.control_mode == 'corridor':
                self.corridor = SafeCorridor(self.get_parameter('corridor_file').value,
                                            whole_body_radius(share/'models/my/arm.urdf'))
                self.corridor.plan(self.body_target, self.body_target)
        self.state = State(); self.pose = self.q = None
        self.pose_wall = self.joint_wall = -1e9
        self.pose_sim = self.joint_sim = -1e9
        self.phase = 'WAIT'; self.start = time.monotonic(); self.last_sim = None
        self.anchor = None; self.align = np.eye(3); self.yaw0 = 0.
        self.p_ref = None; self.q_ref = self.home.copy(); self.yaw_ref = 0.
        self.last_request = 0.; self.pending = None; self.configured = False
        self.last_report = 0.; self.last_R = None; self.omega = np.zeros(3)
        self.hold_since = None; self.abort = False
        self.lio_wall = -1e9; self.lio_good_since = None; self.last_lio = None; self.lio_anchor = None; self.lio_rotation = np.eye(3)
        if self.interactive and self.control_mode == 'wholebody':
            self.create_subscription(PoseStamped, '/ee_goal', self.on_goal, 10)
            self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_2d, 10)
            self.create_subscription(PoseStamped, '/wholebody/body_goal', self.on_wholebody_body_goal, 10)
            self.create_subscription(Bool, '/wholebody/motion_enabled', self.on_motion_enabled, 10)
        if self.control_mode != 'wholebody':
            self.create_subscription(PoseStamped, '/body_goal', self.on_body_goal, 10)
            self.create_subscription(PoseStamped, '/goal_pose', self.on_body_goal_2d, 10)
        if self.control_mode == 'manual':
            self.create_subscription(TwistStamped, '/manual/body_velocity', self.on_velocity, 10)
            self.create_subscription(JointState, '/manual/arm_goal', self.on_arm_goal, 10)
        self.create_subscription(Odometry, '/Odometry', self.on_lio, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(PoseStamped, '/mavros/mavros/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(JointState, '/my/arm/joint_states', self.on_joint, qos_profile_sensor_data)
        self.arm_pub = self.create_publisher(JointState, '/my/arm/position_command', 1)
        self.body_pubs = [self.create_publisher(PoseStamped, t, 1) for t in
                          ('/mavros/mavros/local', '/mavros/setpoint_position/local')]
        self.report = self.create_publisher(String, '/wall_control/telemetry', 10)
        self.command_report = self.create_publisher(String, '/wall_control/command_status', 10)
        self.mode = self.create_client(SetMode, '/mavros/set_mode')
        self.arm = self.create_client(CommandBool, '/mavros/mavros/arming')
        self.param = self.create_client(ParamSetV2, '/mavros/mavros/set')
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def ready_for_goal(self):
        if self.abort or self.phase not in ('APPROACH', 'HOLD'):
            raise ValueError('Wait for takeoff to finish (APPROACH/HOLD) before sending a goal')

    def command_status(self, text):
        self.command_report.publish(String(data=text))

    def on_body_goal_2d(self, msg):
        msg.pose.position.z = self.takeoff_height
        self.on_body_goal(msg)

    def on_motion_enabled(self, msg):
        if self.abort:
            self.command_status('Body motion REJECTED: task aborted'); return
        self.motion_enabled = msg.data
        self.command_status('Body motion '+('enabled' if msg.data else 'paused'))

    def on_wholebody_body_goal(self, msg):
        try:
            self.ready_for_goal()
            if not self.motion_started and self.phase != 'HOLD':
                raise ValueError('Wait for the end effector to settle before moving the body reference')
            if msg.header.frame_id != 'map': raise ValueError('Body reference frame must be map')
            xyz = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            q = msg.pose.orientation
            quat = np.array([q.x, q.y, q.z, q.w])
            if not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < .5:
                raise ValueError('Invalid body orientation')
            yaw = Rotation.from_quat(quat).as_euler('xyz')[2]
            self.motion.set_offset(np.r_[xyz-self.body_center, yaw], np.r_[self.body_target-self.body_center, self.yaw_ref])
        except ValueError as exc:
            self.command_status('Whole-body reference REJECTED: '+str(exc)); return
        self.motion_time = 0.  # smooth ramp to the new bounded reference
        self.motion_started = True; self.motion_enabled = True
        self.command_status('Bounded body reference accepted; EE target unchanged')

    def on_body_goal(self, msg):
        try:
            self.ready_for_goal()
            target = validate_goal(msg.header.frame_id,
                                   [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            q = msg.pose.orientation
            quat = np.array([q.x, q.y, q.z, q.w])
            if not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < .5:
                raise ValueError('Body goal needs a finite, nonzero quaternion')
            yaw = Rotation.from_quat(quat).as_euler('xyz')[2]
            if self.corridor is not None:
                if self.measured_position is None:
                    raise ValueError('No measured starting position')
                self.corridor.plan(self.measured_position, target)
                yaw = 0.  # fixed transport attitude
        except ValueError as exc:
            self.command_status('Body goal REJECTED: '+str(exc))
            self.get_logger().warning('Body goal rejected: '+str(exc)); return
        self.body_target = target; self.manual_yaw_goal = yaw
        self.manual_velocity[:] = 0.; self.velocity_received = -1e9
        self.target = target+self.tip_offset
        self.phase = 'APPROACH'; self.hold_since = None
        self.command_status('Body goal accepted: '+str(target.tolist()))

    def on_velocity(self, msg):
        try:
            self.ready_for_goal()
            v, w = msg.twist.linear, msg.twist.angular
            values = np.array([v.x, v.y, v.z, w.z])
            age = self.get_clock().now().nanoseconds*1e-9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
            if msg.header.frame_id != 'map' or not np.all(np.isfinite(values)) or not -.05 <= age <= .3:
                raise ValueError('Velocity needs map frame, finite values and fresh simulation stamp')
            if w.x != 0. or w.y != 0.:
                raise ValueError('Roll/pitch velocity is not an actuated manual command')
        except ValueError as exc:
            self.command_status('Velocity REJECTED: '+str(exc))
            self.get_logger().warning('Velocity rejected: '+str(exc)); return
        self.manual_velocity = np.clip(values, [-.18, -.18, -.15, -.3], [.18, .18, .15, .3])
        self.velocity_received = time.monotonic()

    def on_arm_goal(self, msg):
        try:
            self.ready_for_goal()
            if len(msg.name) != 7 or set(msg.name) != set(NAMES) or len(msg.position) != 7 or msg.velocity or msg.effort:
                raise ValueError('Arm goal requires all seven unique names and positions only')
            q = np.array([msg.position[msg.name.index(n)] for n in NAMES])
            if not np.all(np.isfinite(q)) or np.any(q < self.model.lower) or np.any(q > self.model.upper):
                raise ValueError('Arm goal violates finite joint limits')
        except ValueError as exc:
            self.command_status('Arm goal REJECTED: '+str(exc))
            self.get_logger().warning('Arm goal rejected: '+str(exc)); return
        self.arm_goal = q
        self.command_status('Arm goal accepted')

    def independent_step(self, p, dt):
        if self.corridor is not None:
            self.p_ref, settled = self.corridor.step(p, dt)
        else:
            active = time.monotonic()-self.velocity_received < .3
            if active:
                self.body_target = np.clip(p+.7*self.manual_velocity[:3], [-1., -1.5, .8], [3., 1.5, 2.])
                self.manual_yaw_goal = self.yaw_ref+self.manual_velocity[3]*dt
            elif self.velocity_received > 0.:
                # On command loss hold the measured pose, not an accumulated lead.
                self.body_target = p.copy(); self.manual_yaw_goal = self.yaw_ref
                self.velocity_received = -1e9
            self.p_ref = p+np.clip(self.body_target-p, [-.126, -.126, -.105], [.126, .126, .105])
            yaw_goal = getattr(self, 'manual_yaw_goal', 0.)
            yaw_error = np.arctan2(np.sin(yaw_goal-self.yaw_ref), np.cos(yaw_goal-self.yaw_ref))
            self.yaw_ref += np.clip(yaw_error, -.3*dt, .3*dt)
            settled = np.linalg.norm(p-self.body_target) < .04 and abs(yaw_error) < .03 and np.max(np.abs(self.q-self.arm_goal)) < .025
        step = np.minimum(self.model.speed*.4, [.25, .25, .04, .3, .3, .03, .03])*dt
        self.q_ref += np.clip(self.arm_goal-self.q_ref, -step, step)
        self.q_ref = np.clip(self.q_ref, self.q-.06, self.q+.06)
        self.q_ref = np.clip(self.q_ref, self.model.lower, self.model.upper)
        if self.control_mode == 'manual':
            goal_R = Rotation.from_euler('z', getattr(self, 'manual_yaw_goal', 0.)).as_matrix()
            self.target = self.fk_goal_model.evaluate(self.body_target, goal_R, self.arm_goal)[0]
        else:
            self.target = self.body_target+self.tip_offset
        return settled

    def on_goal_2d(self, msg):
        # RViz's 2D Goal Pose lies on the ground; supply the configured EE height.
        msg.pose.position.z = float(self.get_parameter('goal_height').value)
        self.on_goal(msg)

    def on_goal(self, msg):
        try:
            target = validate_goal(msg.header.frame_id,
                                   [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            self.ready_for_goal()
        except ValueError as exc:
            self.command_status('EE goal REJECTED: '+str(exc))
            self.get_logger().warning('Goal rejected: '+str(exc))
            return
        self.target = target
        self.body_target = target-self.tip_offset
        self.body_center = self.body_target.copy()
        self.motion_started = False; self.motion_time = 0.; self.motion.offset[:] = 0.; self.motion.start_offset[:] = 0.
        self.hold_since = None
        self.phase = 'APPROACH'
        self.get_logger().info('Accepted end-effector goal: '+str(target.tolist()))
        self.command_status('EE goal accepted: '+str(target.tolist()))

    def on_lio(self, msg):
        p = msg.pose.pose.position; v = msg.twist.twist.linear
        xyz = np.array([p.x, p.y, p.z]); velocity = np.array([v.x, v.y, v.z])
        wall = time.monotonic()
        good = (np.all(np.isfinite(xyz)) and np.all(np.isfinite(velocity)) and
                np.linalg.norm(xyz) < 6. and np.linalg.norm(velocity) < 1.5 and
                (self.last_lio is None or np.linalg.norm(xyz-self.last_lio) < .25))
        self.last_lio = xyz
        quat = msg.pose.pose.orientation
        quaternion = np.array([quat.x, quat.y, quat.z, quat.w])
        if not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < .5:
            good = False
        if good:
            self.lio_wall = wall
            if self.lio_good_since is None: self.lio_good_since = wall
        else: self.lio_good_since = None

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        if np.all(np.isfinite(values)) and np.linalg.norm(values[3:]) > .5:
            self.pose = values; self.pose_wall = time.monotonic()
            self.pose_sim = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def on_joint(self, msg):
        if not all(n in msg.name for n in NAMES) or len(msg.position) != len(msg.name): return
        q = np.array([msg.position[msg.name.index(n)] for n in NAMES])
        if np.all(np.isfinite(q)):
            self.q = q; self.joint_wall = time.monotonic()
            self.joint_sim = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def request(self, client, req):
        if client.service_is_ready() and self.pending is None:
            self.pending = client.call_async(req); self.last_request = time.monotonic()

    def tick(self):
        wall = time.monotonic(); now = self.get_clock().now(); sim = now.nanoseconds*1e-9
        if self.pending is not None and self.pending.done():
            try:
                result = self.pending.result()
                self.get_logger().info('Service response: '+str(result))
                if isinstance(result, ParamSetV2.Response): self.configured = bool(result.success)
            except Exception as exc: self.get_logger().warning(str(exc))
            self.pending = None
        fresh = (self.pose is not None and self.q is not None and
                 wall-self.lio_wall < .6 and self.lio_good_since is not None and
                 wall-self.lio_good_since > 3. and wall-max(self.pose_wall, 0.) < .6 and wall-max(self.joint_wall, 0.) < .6 and
                 -.1 <= sim-self.pose_sim < .6 and -.1 <= sim-self.joint_sim < .3)
        if not fresh:
            if self.phase not in ('WAIT', 'PREARM'):
                if self.phase != 'STALE': self.command_status('FAULT: feedback stale; requesting AUTO.LAND')
                self.phase = 'STALE'; self.abort = True
                if wall-self.last_request > 2.:
                    self.request(self.mode, SetMode.Request(custom_mode='AUTO.LAND'))
            return  # stop offboard stream; PX4 also has its own setpoint watchdog
        local_R = Rotation.from_quat(self.pose[3:]).as_matrix()
        if self.anchor is None or (self.phase in ('WAIT', 'PREARM') and not self.state.armed):
            self.anchor = self.pose[:3].copy()
            self.lio_anchor = self.last_lio.copy()
            self.yaw0 = np.arctan2(local_R[1, 0], local_R[0, 0])
            self.align = Rotation.from_euler('z', -self.yaw0).as_matrix()
        R = self.align @ local_R
        p = self.last_lio-self.lio_anchor+self.spawn - (R-np.eye(3)) @ np.array([-.011, -.02329, .12412])
        self.measured_rotation = R.copy()
        dt = .02 if self.last_sim is None else np.clip(sim-self.last_sim, 0., .05)
        if dt > .0001 and self.measured_position is not None:
            alpha = dt/(.3+dt)
            self.measured_velocity = (1-alpha)*self.measured_velocity+alpha*(p-self.measured_position)/dt
        self.measured_position = p.copy()
        if self.last_sim is not None and sim < self.last_sim:
            self.abort = True; self.phase = 'CLOCK_RESET'
        self.last_sim = sim
        if dt > .0001 and self.last_R is not None:
            omega = Rotation.from_matrix(R@self.last_R.T).as_rotvec()/dt
            self.omega = .85*self.omega+.15*omega
        self.last_R = R.copy()
        if self.p_ref is None: self.p_ref = p.copy()
        tip, tool_R, _ = self.model.evaluate(p, R, self.q)
        solve_ms = 0.
        if self.phase == 'WAIT': self.phase = 'PREARM'
        if self.phase == 'PREARM':
            self.p_ref = p.copy()
            if wall-self.last_request > 2. and self.state.connected:
                if not self.configured:
                    req = ParamSetV2.Request(); req.param_id = 'MPC_THR_HOVER'
                    req.value = ParameterValue(type=3, double_value=float(self.get_parameter('hover_thrust').value))
                    self.request(self.param, req)
                elif wall-self.start > self.get_parameter('settle_seconds').value and self.get_parameter('auto_arm').value:
                    if self.state.mode != 'OFFBOARD': self.request(self.mode, SetMode.Request(custom_mode='OFFBOARD'))
                    elif not self.state.armed: self.request(self.arm, CommandBool.Request(value=True))
            if self.state.armed and self.state.mode == 'OFFBOARD':
                self.phase = 'TAKEOFF'; self.takeoff_started = sim; self.get_logger().info('TAKEOFF')
        elif self.abort or self.state.mode != 'OFFBOARD' or not self.state.armed:
            if self.phase != 'ABORT': self.command_status('ABORT: flight mode changed or vehicle disarmed')
            self.phase = 'ABORT'; self.abort = True
            return  # do not automatically re-arm or fight pilot/failsafe mode changes
        elif self.phase == 'TAKEOFF':
            if sim-self.takeoff_started > 30.:
                self.abort = True; self.phase = 'TAKEOFF_TIMEOUT'
                self.get_logger().error('Takeoff did not converge within 30 simulation seconds')
                self.command_status('FAULT: takeoff timeout; requesting AUTO.LAND')
                self.request(self.mode, SetMode.Request(custom_mode='AUTO.LAND')); return
            goal = np.array([self.spawn[0], self.spawn[1], self.takeoff_height])
            self.p_ref = goal.copy()  # PX4 owns takeoff acceleration/ramp; avoid ground integrator windup.
            if np.linalg.norm(p-goal) < .06:
                self.phase = 'APPROACH'; self.get_logger().info('APPROACH')
        elif self.phase in ('APPROACH', 'HOLD') and dt > 0.:
            started = time.perf_counter()
            try:
                if self.control_mode != 'wholebody':
                    settled = self.independent_step(p, dt)
                else:
                    self.wholebody_step(p, R, dt)
                    settled = np.linalg.norm(tip-self.target) < .025 and np.linalg.norm(p-self.body_target) < .06
            except (ValueError, RuntimeError) as exc:
                self.command_status('FAULT: '+str(exc))
                self.get_logger().error(str(exc)); self.abort = True; self.phase = 'SOLVER_FAULT'
                self.request(self.mode, SetMode.Request(custom_mode='AUTO.LAND')); return
            solve_ms = (time.perf_counter()-started)*1000.
            self.hold_since = sim if settled and self.hold_since is None else self.hold_since if settled else None
            self.phase = 'HOLD' if self.hold_since is not None and sim-self.hold_since > 2. else 'APPROACH'
        self.publish_control(now, wall, sim, p, tip, tool_R, solve_ms)

    def wholebody_step(self, p, R, dt):
        body_velocity = np.zeros(4); body_yaw = 0.; bounds = None
        ready = (np.all(np.abs(p-self.body_center) < .5*self.motion.bounds[:3])
                 and np.linalg.norm(self.measured_velocity) < .025)
        if self.motion_enabled and not self.motion_started and self.phase == 'HOLD' and ready:
            self.motion.set_offset(np.zeros(4), np.r_[p-self.body_center, self.yaw_ref])
            self.motion_started = True; self.motion_time = 0.
            self.command_status('EE settled; starting bounded body motion')
        if self.motion_started:
            tip_error = np.linalg.norm(self.model.evaluate(p, R, self.q)[0]-self.target)
            self.motion_paused = bool(not self.motion_enabled or tip_error > .04)
            if not self.motion_paused: self.motion_time += dt
            offset, body_velocity = self.motion.sample(self.motion_time)
            if self.motion_paused: body_velocity[:] = 0.
            self.body_target = self.body_center+offset[:3]; body_yaw = offset[3]
            bounds = np.array([self.body_center-self.motion.bounds[:3], self.body_center+self.motion.bounds[:3]])
        precision = (self.measured_motion_compensation and
                     np.linalg.norm(self.model.evaluate(p, R, self.q)[0]-self.target) < .08)
        velocity, tip, error = self.control.solve(p, R, self.q, self.target, self.body_target, dt, self.omega,
                                                body_yaw=body_yaw, body_velocity=body_velocity,
                                                body_bounds=bounds, fixed_tip=self.motion_started,
                                                measured_body_velocity=self.measured_velocity if precision else None)
        # PX4 already integrates position error into flight commands.
        # A short velocity lookahead avoids a second position integrator
        # accumulating lead while the heavy vehicle is still accelerating.
        self.p_ref = p + .7*velocity[:3] - self.velocity_damping*self.measured_velocity
        if self.motion_started:
            # The kinematic solver supplies a desired velocity, not an
            # acceleration. Brake measured momentum when converting it to a
            # PX4 position command, otherwise arm reaction and flight-loop
            # lag can amplify a small periodic reference beyond the box.
            self.p_ref = p+1.4*velocity[:3]-.7*self.measured_velocity
        if bounds is not None: self.p_ref = np.clip(self.p_ref, bounds[0], bounds[1])
        if not self.interactive:
            self.p_ref[0] = min(self.p_ref[0], self.target[0]-.39)
        yaw_limit = self.motion.bounds[3] if self.motion_started else .2
        self.yaw_ref = np.clip(self.yaw_ref+velocity[3]*dt, -yaw_limit, yaw_limit)
        self.q_ref[:5] += velocity[4:]*dt
        self.q_ref[:5] = np.clip(self.q_ref[:5], self.q[:5]-.06, self.q[:5]+.06)
        self.q_ref = np.clip(self.q_ref, self.model.lower+.0001, self.model.upper-.0001)

    def publish_control(self, now, wall, sim, p, tip, tool_R, solve_ms):
        arm = JointState(); arm.header.stamp = now.to_msg(); arm.name = NAMES
        arm.position = self.q_ref.tolist(); arm.effort = self.model.gravity().tolist()
        self.arm_pub.publish(arm)
        body = PoseStamped(); body.header.stamp = now.to_msg(); body.header.frame_id = 'map'
        reference = self.pose[:3] + self.align.T @ (self.p_ref-p)
        body.pose.position.x, body.pose.position.y, body.pose.position.z = map(float, reference)
        quat = Rotation.from_euler('z', self.yaw_ref+self.yaw0).as_quat()
        body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z, body.pose.orientation.w = map(float, quat)
        for pub in self.body_pubs: pub.publish(body)
        if wall-self.last_report > .2:
            payload = dict(t=sim, phase=self.phase, body=p.tolist(), body_target=self.body_target.tolist(),
                           tip=tip.tolist(), target=self.target.tolist(), q=self.q.tolist(),
                           ee_error=float(np.linalg.norm(tip-self.target)), body_error=float(np.linalg.norm(p-self.body_target)),
                           tool_angle=float(Rotation.from_matrix(tool_R).magnitude()), solve_ms=solve_ms,
                           armed=self.state.armed, mode=self.state.mode)
            payload['control_mode'] = self.control_mode
            payload['body_orientation'] = Rotation.from_matrix(self.measured_rotation).as_quat().tolist()
            payload['body_reference'] = self.p_ref.tolist()
            payload['joint_target'] = self.q_ref.tolist()
            payload['joint_names'] = NAMES
            payload['joint_goal'] = self.arm_goal.tolist() if self.control_mode == 'manual' else self.q_ref.tolist()
            payload['body_target_yaw'] = float(getattr(self, 'manual_yaw_goal', 0.)) if self.control_mode == 'manual' else float(self.yaw_ref)
            if self.control_mode == 'wholebody':
                payload['wholebody'] = dict(center=self.body_center.tolist(), bounds=self.motion.bounds.tolist(),
                                            offset=self.motion.offset.tolist(), started=self.motion_started,
                                            enabled=self.motion_enabled, paused=self.motion_paused,
                                            motion_time=self.motion_time, period=self.motion.period)
            if self.corridor is not None:
                payload['corridor'] = dict(cells=self.corridor.cells.tolist(), raw_cells=self.corridor.raw.tolist(),
                                          route=[v.tolist() for v in self.corridor.waypoints],
                                          start=self.corridor.start.tolist(),
                                          route_cells=self.corridor.route_cells,
                                          index=self.corridor.index, radius=self.corridor.radius,
                                          margin=self.corridor.margin, smooth_path=self.corridor.smooth_path,
                                          trajectory_time=self.corridor.elapsed, trajectory_duration=self.corridor.trajectory.duration,
                                          trajectory_paused=self.corridor.paused,
                                          velocity=self.corridor.velocity.tolist(), acceleration=self.corridor.acceleration.tolist())
            self.report.publish(String(data=json.dumps(payload))); self.last_report = wall
            if int(wall)%5 == 0: self.get_logger().info(f'{self.phase}: ee={payload["ee_error"]:.3f} body={payload["body_error"]:.3f} solve={solve_ms:.1f}ms')


def main():
    rclpy.init(); node = WallControl()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
