"""Offscreen Qt interaction and real ROS delivery for the manual slider panel."""
import json
import time

import numpy as np
from PyQt5 import QtCore, QtTest, QtWidgets
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from scipy.spatial.transform import Rotation

from check_ros_goals import load
from astra_wall_control.geometry import NAMES


def main():
    rclpy.init()
    app = QtWidgets.QApplication([])
    ui = load('manual_sliders').ManualSliders(app)
    peer = Node('slider_test_peer')
    arms, bodies = [], []
    peer.create_subscription(JointState, '/manual/arm_goal', arms.append, 10)
    peer.create_subscription(PoseStamped, '/body_goal', bodies.append, 10)
    telemetry = peer.create_publisher(String, '/wall_control/telemetry', 10)
    row = dict(control_mode='manual', phase='HOLD', armed=True, mode='OFFBOARD',
               body_target=[.2, 0., 1.6], body_target_yaw=.1,
               joint_goal=[0., -.05, .1, .05, 0., .01, .01], tip=[.6, 0., 1.4])

    def pump(seconds=.15, feedback=True):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if feedback:
                telemetry.publish(String(data=json.dumps(row)))
            app.processEvents()
            rclpy.spin_once(peer, timeout_sec=.005)
            ui.pump()

    try:
        assert not ui.editors.isEnabled()
        ui.send_arm(); ui.send_body()
        pump(.2, feedback=False)
        assert not arms and not bodies
        pump(1.)
        assert ui.editors.isEnabled()
        np.testing.assert_allclose([v.value() for v in ui.arm], row['joint_goal'])
        assert not arms and not bodies  # initialization must not command flight
        for i, editor in enumerate(ui.arm):
            previous = len(arms)
            before = ui.fk.text()
            editor.slider.setSliderDown(True)
            editor.slider.setValue(7500)
            pump()
            assert len(arms) == previous  # drag is only a preview
            if i < 5:
                assert ui.fk.text() != before
            editor.slider.setSliderDown(False)
            pump()
            assert len(arms) == previous+1
            assert arms[-1].name == NAMES and not arms[-1].velocity and not arms[-1].effort
            np.testing.assert_allclose(arms[-1].position, [v.value() for v in ui.arm])
        # Full travel at both endpoints, especially roll's +/- pi/2 limit.
        for endpoint in (0, 10000):
            for editor in ui.arm:
                editor.slider.setValue(endpoint)
                pump(.03)
                assert np.all(np.array(arms[-1].position) >= ui.model.lower)
                assert np.all(np.array(arms[-1].position) <= ui.model.upper)
        for editor in ui.body:
            previous = len(bodies)
            editor.slider.setSliderDown(True); editor.slider.setValue(6000)
            pump(); assert len(bodies) == previous
            editor.slider.setSliderDown(False)
            pump(); assert len(bodies) == previous+1
        body = bodies[-1]
        assert body.header.frame_id == 'map'
        np.testing.assert_allclose([body.pose.position.x, body.pose.position.y, body.pose.position.z],
                                   [v.value() for v in ui.body[:3]])
        q = body.pose.orientation
        np.testing.assert_allclose(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2], ui.body[3].value())
        # Keyboard slider steps and numeric entry must also publish.
        previous = len(arms)
        QtTest.QTest.keyClick(ui.arm[0].slider, QtCore.Qt.Key_Left)
        pump(); assert len(arms) == previous+1
        ui.arm[0].number.setFocus(); ui.arm[0].number.selectAll()
        QtTest.QTest.keyClicks(ui.arm[0].number, '0.12')
        QtTest.QTest.keyClick(ui.arm[0].number, QtCore.Qt.Key_Return)
        pump(); assert abs(arms[-1].position[0]-.12) < 1e-6
        previous = len(arms)
        ui.sync_targets(); pump(); assert len(arms) == previous
        pump(1.1, feedback=False)
        assert not ui.editors.isEnabled()
        ui.send_arm(); pump(.1, feedback=False); assert len(arms) == previous
        for field, value in [('phase', 'TAKEOFF'), ('control_mode', 'wholebody'),
                             ('armed', False), ('mode', 'POSCTL')]:
            original = row[field]; row[field] = value
            pump(); ui.send_arm(); ui.send_body(); pump()
            assert not ui.editors.isEnabled() and len(arms) == previous
            row[field] = original
        print('PASS: 11 sliders, FK preview, ROS commands, endpoints, keyboard, startup and stale/mode locks')
    finally:
        ui.timer.stop(); ui.window.close(); ui.destroy_node(); peer.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
