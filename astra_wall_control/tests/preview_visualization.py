"""Replay recorded telemetry into RViz without starting or commanding a vehicle.

Use a separate ROS_DOMAIN_ID. The interactive target is deliberately read-only.
"""
import argparse
import json
import gzip
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from PyQt5 import QtWidgets
from std_msgs.msg import String
from check_ros_goals import load


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--scene', choices=['air', 'corridor'], default='corridor')
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--speed', type=float, default=20.)
    parser.add_argument('--capture-dir', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = Path(args.input)
    rows = json.loads(gzip.decompress(source.read_bytes()) if source.suffix == '.gz' else source.read_text())['samples']
    if not rows: raise SystemExit('No samples in input')
    mode = rows[-1].get('control_mode', 'wholebody')
    rclpy.init(args=['--ros-args', '-p', 'goal_kind:='+('ee' if mode == 'wholebody' else 'body'),
                    '-p', 'control_mode:='+mode,
                    '-p', 'enable_goal_input:=false', '-p', 'scene_file:='+str(root/'worlds'/ (args.scene+'.world'))])
    ui = load('air_goal').AirGoal()
    ui.renderer.command_status = 'RECORDED REPLAY - flight input disabled'
    app = QtWidgets.QApplication([]); dashboard = load('flight_dashboard').Dashboard(app)
    dashboard.events.appendPlainText('只读历史回放：飞行目标输入已禁用。')
    if 'body_orientation' not in rows[0]:
        dashboard.events.appendPlainText('旧记录缺少机体姿态，回放模型使用单位姿态；新遥测已包含实际姿态。')
    config = 'air_test.rviz' if args.scene == 'corridor' else 'manipulation.rviz'
    process = subprocess.Popen(['rviz2', '-d', str(root/'config'/config)], start_new_session=True)
    print('RVIZ_PID='+str(process.pid), flush=True)
    print('DASHBOARD_WINDOW='+str(int(dashboard.window.winId())), flush=True)
    start = time.monotonic(); index = 0
    try:
        while time.monotonic()-start < args.seconds and process.poll() is None:
            t = rows[0]['t']+(time.monotonic()-start)*args.speed
            while index+1 < len(rows) and rows[index+1]['t'] <= t:
                index += 1
                message = String(data=json.dumps(rows[index]))
                ui.telemetry(message); dashboard.telemetry(message)
            # Keep the final frame visible after playback reaches the end.
            message = String(data=json.dumps(rows[index]))
            ui.telemetry(message); dashboard.telemetry(message)
            rclpy.spin_once(ui, timeout_sec=.03)
            app.processEvents()
        if args.capture_dir:
            args.capture_dir.mkdir(parents=True, exist_ok=True)
            dashboard.window.grab().save(str(args.capture_dir/'dashboard.png'))
            # Restrict capture to windows owned by the RViz process we started.
            windows = subprocess.check_output(['xdotool', 'search', '--onlyvisible', '--pid', str(process.pid)], text=True).split()
            for window in windows:
                title = subprocess.check_output(['xdotool', 'getwindowname', window], text=True)
                if 'RViz' in title:
                    subprocess.run(['import', '-window', window, str(args.capture_dir/'rviz.png')], check=True)
                    break
            else:
                raise RuntimeError('No visible RViz window owned by the replay process')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate(); process.wait(timeout=5)
        dashboard.timer.stop(); dashboard.window.close(); dashboard.destroy_node()
        ui.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
