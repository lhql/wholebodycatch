#!/usr/bin/python3
"""Exercise the RViz goal topic against SITL and independently score Gazebo truth."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from astra_wall_control.controller import ArmModel
from astra_wall_control.geometry import NAMES


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=260.)
    parser.add_argument('--output', default='/tmp/astra_air_goals.json')
    args = parser.parse_args()
    rclpy.init(); node = Node('sitl_air_goal_validation')
    model = ArmModel(get_package_share_directory('astra_wall_control')+'/models/my/arm.urdf')
    pub = node.create_publisher(PoseStamped, '/ee_goal', 10)
    goals = [[2., 0., 1.4], [1., .5, 1.6], [1.5, -.5, 1.2]]
    state = dict(truth=None, q=None, truth_wall=-1e9, joint_wall=-1e9,
                 stage=0, since=None, previous_t=None, passed=[])
    rows = []
    def truth(msg):
        state['truth'] = msg.pose.pose; state['truth_wall'] = time.monotonic()
    def joints(msg):
        if all(n in msg.name for n in NAMES):
            state['q'] = np.array([msg.position[msg.name.index(n)] for n in NAMES])
            state['joint_wall'] = time.monotonic()
    def telemetry(msg):
        row = json.loads(msg.data)
        stage = state['stage']
        if stage >= len(goals) or state['truth'] is None or state['q'] is None:
            return
        pose = state['truth']; quat = pose.orientation
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        R = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
        tip = model.evaluate(p, R, state['q'])[0]
        row.update(stage=stage, truth_body=p.tolist(), truth_tip=tip.tolist(),
                   truth_ee_error=float(np.linalg.norm(tip-goals[stage])),
                   truth_body_error=float(np.linalg.norm(p-row['body_target'])))
        rows.append(row)
        fresh = time.monotonic()-min(state['truth_wall'], state['joint_wall']) < .3
        continuous = state['previous_t'] is None or 0. < row['t']-state['previous_t'] < .6
        state['previous_t'] = row['t']
        good = (fresh and continuous and row['phase']=='HOLD' and
                np.allclose(row['target'], goals[stage]) and row['truth_ee_error'] < .035 and
                row['truth_body_error'] < .08)
        if not good:
            state['since'] = None
            return
        if state['since'] is None:
            state['since'] = row['t']
        if row['t']-state['since'] < 10.:
            return
        tail = [r for r in rows if r['stage']==stage and r['t'] >= state['since']]
        state['passed'].append(dict(goal=goals[stage], hold_seconds=row['t']-state['since'],
                                    max_ee_error=max(r['truth_ee_error'] for r in tail),
                                    max_body_error=max(r['truth_body_error'] for r in tail)))
        state['stage'] += 1; state['since'] = None
        print('Verified:', state['passed'][-1], flush=True)
        if state['stage'] < len(goals):
            goal = PoseStamped(); goal.header.frame_id = 'map'
            goal.header.stamp = node.get_clock().now().to_msg(); goal.pose.orientation.w = 1.
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = goals[state['stage']]
            pub.publish(goal)
            print('Sent goal:', goals[state['stage']], flush=True)
    node.create_subscription(Odometry, '/wall_eval/body', truth, qos_profile_sensor_data)
    node.create_subscription(JointState, '/my/arm/joint_states', joints, qos_profile_sensor_data)
    node.create_subscription(String, '/wall_control/telemetry', telemetry, 10)
    start = time.monotonic()
    try:
        while time.monotonic()-start < args.seconds and state['stage'] < len(goals):
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    result = dict(passed=state['stage']==len(goals), stages=state['passed'], samples=rows)
    Path(args.output).write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='samples'}, indent=2))
    node.destroy_node(); rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
