#!/usr/bin/python3
"""Run in a separate terminal after manual_flight.launch.py (simulation time)."""
import curses
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from astra_wall_control.controller import ArmModel
from astra_wall_control.geometry import NAMES


class Teleop(Node):
    def __init__(self):
        super().__init__('manual_teleop')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.velocity = self.create_publisher(TwistStamped, '/manual/body_velocity', 10)
        self.arm = self.create_publisher(JointState, '/manual/arm_goal', 10)
        self.create_subscription(String, '/wall_control/telemetry', self.feedback, 10)
        self.model = ArmModel(Path(get_package_share_directory('astra_wall_control'))/'models/my/arm.urdf')
        self.row = None; self.q = None; self.received = -1e9

    def feedback(self, msg):
        self.row = json.loads(msg.data); self.received = time.monotonic()
        if self.q is None:
            self.q = np.array(self.row['q'])

    def send_velocity(self, values):
        msg = TwistStamped(); msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.z = map(float, values)
        self.velocity.publish(msg)

    def run(self, screen):
        screen.nodelay(True); selected = 0; velocity = np.zeros(4); last_key = -1e9
        # ENU/map directions, independent of current aircraft yaw.
        keys = {'i': (0, .15), 'k': (0, -.15), 'j': (1, .15), 'l': (1, -.15),
                'r': (2, .12), 'f': (2, -.12), 'u': (3, .25), 'o': (3, -.25)}
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=.02)
                code = screen.getch(); key = chr(code) if 0 <= code < 256 else ''
                ready = (self.row is not None and time.monotonic()-self.received < 1. and
                         self.row.get('control_mode') == 'manual' and self.row['phase'] in ('APPROACH', 'HOLD'))
                if key == 'q': break
                if key in '1234567' and key: selected = int(key)-1
                if key in keys and ready:
                    velocity[:] = 0.; axis, value = keys[key]; velocity[axis] = value
                    last_key = time.monotonic()
                if key == ' ' or not ready or time.monotonic()-last_key > .20:
                    velocity[:] = 0.
                # Only stream while moving or explicitly stopping; this lets RViz
                # body goals remain in control when the terminal is idle.
                if np.any(velocity) or key == ' ':
                    self.send_velocity(velocity)
                if key in ('+', '=', '-') and ready and self.q is not None:
                    delta = .005 if selected in (2, 5, 6) else .03
                    self.q[selected] = np.clip(self.q[selected]+(-delta if key == '-' else delta),
                                               self.model.lower[selected], self.model.upper[selected])
                    msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = NAMES; msg.position = self.q.tolist(); self.arm.publish(msg)
                screen.erase()
                lines = ['Manual aerial manipulator (map/ENU)',
                         'i/k X | j/l Y | r/f Z | u/o yaw | SPACE stop | q quit',
                         '1..7 select joint; +/- move selected joint (3,6,7 metres)',
                         'Selected: '+NAMES[selected],
                         'Status: '+(self.row['phase'] if self.row else 'waiting for telemetry'),
                         'Commands enabled: '+str(ready),
                         'Joint target: '+str(None if self.q is None else np.round(self.q, 3))]
                height, width = screen.getmaxyx()
                for i, line in enumerate(lines[:max(0, height-1)]):
                    screen.addnstr(i, 0, line, max(0, width-1))
                screen.refresh()
        finally:
            self.send_velocity(np.zeros(4))


def main():
    rclpy.init(); node = Teleop()
    try:
        curses.wrapper(node.run)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
