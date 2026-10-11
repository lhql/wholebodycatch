#!/usr/bin/python3
"""Independent Gazebo-truth evaluation, never fed into the controller."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from astra_wall_control.controller import ArmModel
from astra_wall_control.geometry import NAMES


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--seconds',type=float,default=180.)
    parser.add_argument('--output',default='log/wall_validation.json'); args=parser.parse_args()
    rclpy.init(); node=Node('wall_observer')
    model=ArmModel(get_package_share_directory('astra_wall_control')+'/models/my/arm.urdf')
    data={'truth':None,'q':None,'telemetry':None,'last_wall':-1e9,'truth_wall':-1e9,'joint_wall':-1e9}; rows=[]
    output=Path(args.output); output.parent.mkdir(parents=True,exist_ok=True)
    stream=open(str(output)+'.jsonl','w',buffering=1)
    def truth(m): data['truth']=m; data['truth_wall']=time.monotonic()
    def joints(m):
        if all(n in m.name for n in NAMES):
            data['q']=np.array([m.position[m.name.index(n)] for n in NAMES]); data['joint_wall']=time.monotonic()
    def telemetry(m):
        row=json.loads(m.data); data['telemetry']=row; data['last_wall']=time.monotonic()
        row['truth_fresh']=time.monotonic()-min(data['truth_wall'],data['joint_wall'])<.25
        if data['truth'] is not None and data['q'] is not None:
            pose=data['truth'].pose.pose; p=np.array([pose.position.x,pose.position.y,pose.position.z]); q=pose.orientation
            tip,R,_=model.evaluate(p,Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),data['q'])
            row.update(truth_body=p.tolist(),truth_tip=tip.tolist(),truth_ee_error=float(np.linalg.norm(tip-row['target'])),
                       truth_body_error=float(np.linalg.norm(p-row['body_target'])),truth_angle=float(Rotation.from_matrix(R).magnitude()))
        rows.append(row); stream.write(json.dumps(row)+"\n")
    node.create_subscription(Odometry,'/wall_eval/body',truth,qos_profile_sensor_data)
    node.create_subscription(JointState,'/my/arm/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(String,'/wall_control/telemetry',telemetry,10)
    start=time.monotonic()
    try:
        while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:pass
    tail=[r for r in rows if rows and r['t']>rows[-1]['t']-20.]
    summary={'samples':len(rows),'final':rows[-1] if rows else None}
    for key in ('ee_error','body_error','truth_ee_error','truth_body_error','truth_angle','solve_ms'):
        values=[r[key] for r in tail if key in r]
        if values:summary[key]={'rms':float(np.sqrt(np.mean(np.square(values)))),'max':float(max(values)),'p95':float(np.percentile(values,95))}
    summary['passed']=bool(time.monotonic()-data['last_wall']<1. and tail and tail[-1]['t']-tail[0]['t']>18. and all(r['truth_fresh'] and r['phase']=='HOLD' and r.get('truth_ee_error',1.)<.035 and r.get('truth_body_error',1.)<.08 for r in tail))
    path=Path(args.output); path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps({'summary':summary,'samples':rows},indent=2));print(json.dumps(summary,indent=2))
    stream.close();node.destroy_node();rclpy.shutdown()
    return 0 if summary['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
