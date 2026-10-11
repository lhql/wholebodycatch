#!/usr/bin/python3
"""Run one isolated SITL acceptance and clean up only its process group.

Source the built ROS workspace first. PX4 UDP ports still require other SITL
instances to be stopped. All runtime output is written to --output-dir.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['manual', 'corridor', 'wholebody'], required=True)
    parser.add_argument('--seconds', type=float, default=300.)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=75)
    parser.add_argument('--goal-source', choices=['topic', 'rviz'], default='topic')
    parser.add_argument('--motion-periods', type=int, default=2)
    args = parser.parse_args()
    destination = args.output_dir.resolve(); destination.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOG_DIR=str(destination/'ros'),
               ASTRA_WALL_ROOTFS=str(destination/'px4'), OPENBLAS_NUM_THREADS='1',
               GAZEBO_MASTER_URI='http://127.0.0.1:11365')
    launch = dict(manual='manual_flight', corridor='wholebody_corridor', wholebody='wholebody_point')[args.mode]
    extra_args = ['sliders:=false'] if args.mode == 'manual' else []
    with (destination/'launch.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'astra_wall_control', launch+'.launch.py',
                                    'gui:=false', 'rviz:=false']+extra_args, env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return subprocess.call([sys.executable, str(Path(__file__).with_name('sitl_modes.py')),
                                    '--mode', args.mode, '--seconds', str(args.seconds),
                                    '--goal-source', args.goal_source,
                                    '--motion-periods', str(args.motion_periods),
                                    '--output', str(destination/'result.json')], env=env)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=30.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10.)


if __name__ == '__main__':
    raise SystemExit(main())
