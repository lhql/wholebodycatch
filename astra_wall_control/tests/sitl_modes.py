#!/usr/bin/python3
"""Independent physical acceptance for an already running new launch.

Run in the same ROS domain. Sends a goal after takeoff and checks Gazebo truth,
not merely the estimator or receipt of a command. Exits nonzero on failure.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from livox_ros_driver2.msg import CustomMsg
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerFeedback
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from astra_wall_control.controller import ArmModel
from astra_wall_control.geometry import NAMES


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['manual', 'corridor', 'wholebody'], required=True)
    parser.add_argument('--seconds', type=float, default=280.)
    parser.add_argument('--output', required=True)
    parser.add_argument('--goal-source', choices=['topic', 'rviz'], default='topic')
    parser.add_argument('--motion-periods', type=int, default=2)
    args = parser.parse_args()
    if args.motion_periods < 1: parser.error('--motion-periods must be positive')
    rclpy.init(); node = Node('sitl_mode_validation')
    model = ArmModel(Path(get_package_share_directory('astra_wall_control'))/'models/my/arm.urdf')
    target = {'manual': [.5, .2, 1.6], 'corridor': [2.7, 0., 1.6], 'wholebody': [1., .5, 1.4]}[args.mode]
    target = np.array(target)
    arm_target = np.array([.12, -.05, .1, .05, 0., .005, .005])
    pub = node.create_publisher(PoseStamped, '/ee_goal' if args.mode == 'wholebody' else '/body_goal', 10)
    arm_pub = node.create_publisher(JointState, '/manual/arm_goal', 10)
    feedback_pub = node.create_publisher(InteractiveMarkerFeedback, '/air_test/goal/feedback', 10)
    data = dict(pose=None, q=None, truth_time=-1e9, joint_time=-1e9, sent=False,
                since=None, previous=None, passed=False, clearance_ok=True, sensors={}, received=None,
                ready_samples=0, motion_bounds_ok=True)
    rows = []
    motion_rows = []
    scans = set()
    stream = Path(args.output).with_suffix('.jsonl').open('w')

    def sensor(name, msg):
        pose = msg.pose.pose if isinstance(msg, Odometry) else msg.pose
        p, q = pose.position, pose.orientation
        data['sensors'][name] = dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                    position=[p.x, p.y, p.z], orientation=[q.x, q.y, q.z, q.w])

    def truth(msg):
        data['pose'] = msg.pose.pose; data['truth_time'] = time.monotonic()

    def joints(msg):
        if len(msg.position) == len(msg.name) and all(n in msg.name for n in NAMES):
            data['q'] = np.array([msg.position[msg.name.index(n)] for n in NAMES])
            data['joint_time'] = time.monotonic()

    def scan(msg):
        if data['pose'] is None: return
        pose = data['pose']; q = pose.orientation
        key = 'ground' if pose.position.z < .6 else 'air' if pose.position.z > 1.5 else None
        if key is None or key in scans: return
        points = np.array([[p.x, p.y, p.z] for p in msg.points])
        np.savez(Path(args.output).with_name('lidar_'+key+'.npz'), points=points,
                 body=[pose.position.x, pose.position.y, pose.position.z], quat=[q.x, q.y, q.z, q.w])
        scans.add(key)

    def telemetry(msg):
        row = json.loads(msg.data)
        if row.get('control_mode') != args.mode:
            return
        data['received'] = time.monotonic()
        data['ready_samples'] = data['ready_samples']+1 if row['phase'] in ('APPROACH', 'HOLD') else 0
        if not data['sent'] and data['ready_samples'] >= 3:
            goal = PoseStamped(); goal.header.frame_id = 'map'; goal.pose.orientation.w = 1.
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = map(float, target)
            if args.goal_source == 'rviz':
                feedback = InteractiveMarkerFeedback(); feedback.header.frame_id = 'map'
                feedback.client_id = 'sitl_acceptance'; feedback.marker_name = 'ee_goal'; feedback.control_name = 'move_x'
                feedback.pose = goal.pose; feedback.event_type = InteractiveMarkerFeedback.MOUSE_UP
                feedback_pub.publish(feedback)
            else:
                pub.publish(goal)
            if args.mode == 'manual':
                command = JointState(); command.name = NAMES; command.position = arm_target.tolist()
                arm_pub.publish(command)
            data['sent'] = True
            print('Sent {} goal {}'.format(args.mode, target.tolist()), flush=True)
        if data['pose'] is None or data['q'] is None:
            return
        pose = data['pose']; quat = pose.orientation
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        R = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
        tip = model.evaluate(p, R, data['q'])[0]
        measured = tip if args.mode == 'wholebody' else p
        error = float(np.linalg.norm(measured-target))
        row.update(truth_body=p.tolist(), truth_tip=tip.tolist(), acceptance_error=error,
                   truth_orientation=[quat.x, quat.y, quat.z, quat.w], truth_joints=data['q'].tolist(),
                   localization=data['sensors'].copy())
        fresh = time.monotonic()-min(data['truth_time'], data['joint_time']) < .3
        continuous = data['previous'] is None or 0. < row['t']-data['previous'] < .6
        data['previous'] = row['t']
        accepted = np.allclose(row['target' if args.mode == 'wholebody' else 'body_target'], target)
        good = (data['sent'] and accepted and fresh and continuous and row['phase'] in ('APPROACH', 'HOLD')
                and error < (.035 if args.mode == 'wholebody' else .10))
        if args.mode == 'manual':
            row['joint_error'] = float(np.max(np.abs(data['q']-arm_target)))
            good = good and row['joint_error'] < .015
        if args.mode == 'wholebody':
            good = good and np.linalg.norm(p-row['body_target']) < .08
            motion = row.get('wholebody', {})
            good = good and motion.get('started', False) and motion.get('enabled', False)
            offset = np.r_[p-np.asarray(motion.get('center', p)), Rotation.from_matrix(R).as_euler('xyz')[2]]
            row['truth_motion_offset'] = offset.tolist()
            row['truth_motion_in_bounds'] = bool(np.all(np.abs(offset) <= np.asarray(motion.get('bounds', [0.]*4))))
            if accepted and fresh and motion.get('started', False):
                data['motion_bounds_ok'] = data['motion_bounds_ok'] and row['truth_motion_in_bounds']
            good = good and row['truth_motion_in_bounds']
            # A stationary hover must never pass the moving-body task. Require a
            # complete reference periods with actual body AND arm movement.
            if good:
                motion_rows.append(row)
            else:
                motion_rows.clear()
            if motion_rows:
                span = row['t']-motion_rows[0]['t']
                phase_span = motion['motion_time']-motion_rows[0]['wholebody']['motion_time']
                positions = np.array([r['truth_body'] for r in motion_rows])
                angles = np.unwrap([Rotation.from_quat(r['truth_orientation']).as_euler('xyz')[2] for r in motion_rows])
                arm = np.array([r['truth_joints'] for r in motion_rows])
                row['motion_acceptance'] = dict(duration=span, phase_span=phase_span,
                    body_span=np.ptp(positions, axis=0).tolist(), yaw_span=float(np.ptp(angles)),
                    joint_span=np.ptp(arm, axis=0).tolist())
        if args.mode == 'corridor' and row['phase'] in ('APPROACH', 'HOLD'):
            nearest = np.clip(p, [1.2, -.7, 0.], [1.6, .7, 3.15])
            row['truth_obstacle_clearance'] = float(np.linalg.norm(p-nearest)-row['corridor']['radius'])
            data['clearance_ok'] = data['clearance_ok'] and row['truth_obstacle_clearance'] >= 0.
            good = good and data['clearance_ok']
        data['since'] = (row['t'] if data['since'] is None else data['since']) if good else None
        data['passed'] = data['since'] is not None and row['t']-data['since'] >= 8.
        if args.mode == 'wholebody':
            m = row.get('motion_acceptance', {})
            data['passed'] = bool(data['passed'] and data['motion_bounds_ok']
                and m.get('phase_span', 0.) >= args.motion_periods*row['wholebody']['period']
                and min(m.get('body_span', [0.])) > .01 and m.get('yaw_span', 0.) > .04
                and max(m.get('joint_span', [0.])[:5]) > .03)
        rows.append(row)
        stream.write(json.dumps(row)+'\n'); stream.flush()

    node.create_subscription(Odometry, '/wall_eval/body', truth, qos_profile_sensor_data)
    node.create_subscription(JointState, '/my/arm/joint_states', joints, qos_profile_sensor_data)
    node.create_subscription(String, '/wall_control/telemetry', telemetry, 10)
    node.create_subscription(Odometry, '/Odometry', lambda m: sensor('lio', m), qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/mavros/mavros/pose', lambda m: sensor('px4', m), qos_profile_sensor_data)
    node.create_subscription(CustomMsg, '/livox/lidar', scan, qos_profile_sensor_data)
    start = time.monotonic()
    try:
        while time.monotonic()-start < args.seconds and not data['passed']:
            rclpy.spin_once(node, timeout_sec=.2)
            if data['received'] is not None and time.monotonic()-data['received'] > 5.:
                print('FAIL: controller telemetry stopped for 5 seconds', flush=True)
                break
            if not data['motion_bounds_ok']:
                print('FAIL: actual body motion exceeded the configured bounds', flush=True)
                break
    except KeyboardInterrupt:
        pass
    summary = dict(mode=args.mode, goal_source=args.goal_source, passed=data['passed'], goal_sent=data['sent'], samples=len(rows),
                   motion_periods=args.motion_periods if args.mode == 'wholebody' else None,
                   motion_bounds_ok=data['motion_bounds_ok'] if args.mode == 'wholebody' else None,
                   clearance_ok=data['clearance_ok'], final=rows[-1] if rows else None)
    Path(args.output).write_text(json.dumps(dict(summary=summary, samples=rows), indent=2))
    stream.close()
    brief = {k: v for k, v in summary.items() if k != 'final'}
    if rows:
        brief.update(final_phase=rows[-1]['phase'], final_truth_error_m=rows[-1]['acceptance_error'],
                     motion=rows[-1].get('motion_acceptance'))
    print(json.dumps(brief, indent=2))
    node.destroy_node(); rclpy.shutdown()
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
