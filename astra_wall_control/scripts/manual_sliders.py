#!/usr/bin/python3
"""Independent body / seven-joint sliders, with local forward kinematics."""
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np
from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from astra_wall_control.controller import ArmModel
from astra_wall_control.geometry import NAMES


class ValueSlider(QtWidgets.QWidget):
    changed = QtCore.pyqtSignal()
    committed = QtCore.pyqtSignal()

    def __init__(self, label, lower, upper, unit):
        super().__init__()
        self.lower, self.upper = float(lower), float(upper)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        title = QtWidgets.QLabel(label); title.setMinimumWidth(135)
        layout.addWidget(title)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 10000)
        self.slider.setMinimumWidth(180)
        layout.addWidget(self.slider, 1)
        self.number = QtWidgets.QDoubleSpinBox()
        self.number.setDecimals(4)
        self.number.setRange(self.lower, self.upper)
        self.number.setSingleStep(.001 if unit == 'm' else .01)
        self.number.setSuffix(' '+unit)
        self.number.setKeyboardTracking(False)
        self.number.setMinimumWidth(125)
        layout.addWidget(self.number)
        self.slider.valueChanged.connect(self.from_slider)
        self.slider.sliderReleased.connect(self.committed.emit)
        self.number.valueChanged.connect(self.from_number)
        self.number.editingFinished.connect(self.committed.emit)
        self.setToolTip('范围 [{:.4f}, {:.4f}] {}'.format(lower, upper, unit))

    def value(self):
        # Decimal display rounds pi-based URDF endpoints; never send a
        # rounded value a few microradians outside the actual joint limit.
        return float(np.clip(self.number.value(), self.lower, self.upper))

    def set_value(self, value):
        self.number.blockSignals(True)
        self.number.setValue(float(value))
        self.number.blockSignals(False)
        self.slider.blockSignals(True)
        self.slider.setValue(round((self.value()-self.lower)/(self.upper-self.lower)*10000))
        self.slider.blockSignals(False)

    def from_slider(self, value):
        self.set_value(self.lower+(self.upper-self.lower)*value/10000)
        self.changed.emit()
        # Mouse drags commit on release; groove clicks and keyboard steps
        # have no sliderReleased signal and therefore commit here.
        if not self.slider.isSliderDown():
            self.committed.emit()

    def from_number(self, value):
        self.set_value(value)
        self.changed.emit()


class ManualSliders(Node):
    def __init__(self, app):
        super().__init__('manual_sliders')
        share = Path(get_package_share_directory('astra_wall_control'))
        self.model = ArmModel(share/'models/my/arm.urdf')
        self.row = None
        self.received = -1e9
        self.initialized = False
        self.body_pub = self.create_publisher(PoseStamped, '/body_goal', 10)
        self.arm_pub = self.create_publisher(JointState, '/manual/arm_goal', 10)
        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle('Astra · 正运动学滑条控制')
        outer = QtWidgets.QVBoxLayout(self.window)
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(content)
        scroll.setWidget(content); outer.addWidget(scroll)
        intro = QtWidgets.QLabel('拖动横向滑条调整机体和关节；松开后发送。\n'
                                 '也可输入数值并按回车。末端由正运动学计算。')
        intro.setWordWrap(True); layout.addWidget(intro)
        self.status = QtWidgets.QLabel('等待起飞完成'); layout.addWidget(self.status)
        self.editors = QtWidgets.QWidget(); controls = QtWidgets.QVBoxLayout(self.editors)
        controls.setContentsMargins(0, 0, 0, 0)
        self.body = []
        body_group = QtWidgets.QGroupBox('机体目标 · map 坐标系')
        body_layout = QtWidgets.QVBoxLayout(body_group)
        for label, low, high, unit in [('X', -1., 3., 'm'), ('Y', -1.5, 1.5, 'm'),
                                      ('Z', .8, 2., 'm'), ('偏航 yaw', -np.pi, np.pi, 'rad')]:
            slider = ValueSlider(label, low, high, unit)
            slider.changed.connect(self.preview)
            slider.committed.connect(self.send_body)
            body_layout.addWidget(slider); self.body.append(slider)
        controls.addWidget(body_group)
        self.arm = []
        arm_group = QtWidgets.QGroupBox('七关节目标 · URDF 限位内自由组合')
        arm_layout = QtWidgets.QVBoxLayout(arm_group)
        labels = ['1  基座 yaw', '2  肩部 shoulder', '3  伸缩 extension', '4  腕部 wrist',
                  '5  翻滚 roll', '6  左夹指', '7  右夹指']
        for i, label in enumerate(labels):
            unit = 'm' if i in (2, 5, 6) else 'rad'
            slider = ValueSlider(label, self.model.lower[i], self.model.upper[i], unit)
            slider.changed.connect(self.preview)
            slider.committed.connect(self.send_arm)
            arm_layout.addWidget(slider); self.arm.append(slider)
        controls.addWidget(arm_group)
        sync = QtWidgets.QPushButton('读取控制器当前目标（不发送）')
        sync.clicked.connect(self.sync_targets); controls.addWidget(sync)
        self.editors.setEnabled(False); layout.addWidget(self.editors)
        self.fk = QtWidgets.QLabel('FK：等待反馈'); self.fk.setWordWrap(True)
        self.fk.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.fk)
        self.actual = QtWidgets.QLabel('实际末端：等待反馈'); layout.addWidget(self.actual)
        note = QtWidgets.QLabel('FK 预览采用水平机体 + 目标偏航；实际末端包含飞行姿态扰动。\n'
                                'RViz 在目标发送后显示整机目标姿态。关闭面板后保持已接受目标。')
        note.setWordWrap(True); layout.addWidget(note)
        self.events = QtWidgets.QPlainTextEdit(); self.events.setReadOnly(True)
        self.events.setMaximumBlockCount(30); self.events.setMaximumHeight(90)
        layout.addWidget(self.events)
        self.create_subscription(String, '/wall_control/telemetry', self.feedback, 10)
        self.create_subscription(String, '/wall_control/command_status',
                                 lambda msg: self.events.appendPlainText(msg.data), 10)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.pump); self.timer.start(40)
        screen = app.primaryScreen().availableGeometry()
        self.window.resize(min(720, screen.width()), min(850, screen.height()))
        self.window.show()

    def ready(self):
        return (self.row is not None and time.monotonic()-self.received < 1.
                and self.row.get('control_mode') == 'manual'
                and self.row.get('phase') in ('APPROACH', 'HOLD')
                and self.row.get('armed') and self.row.get('mode') == 'OFFBOARD')

    def feedback(self, msg):
        self.row = json.loads(msg.data)
        self.received = time.monotonic()
        if self.ready() and not self.initialized:
            self.sync_targets(); self.initialized = True
        tip = self.row.get('tip')
        if tip is not None:
            self.actual.setText('实际末端 / m：X {:+.4f}  Y {:+.4f}  Z {:+.4f}'.format(*tip))

    def sync_targets(self):
        if not self.ready():
            return
        values = list(self.row['body_target'])+[self.row.get('body_target_yaw', 0.)]
        values[3] = np.arctan2(np.sin(values[3]), np.cos(values[3]))
        for editor, value in zip(self.body+self.arm, values+list(self.row['joint_goal'])):
            editor.set_value(value)
        self.preview()

    def preview(self):
        body = np.array([editor.value() for editor in self.body])
        joints = np.array([editor.value() for editor in self.arm])
        tip, rotation, _ = self.model.evaluate(body[:3], Rotation.from_euler('z', body[3]).as_matrix(), joints)
        quat = Rotation.from_matrix(rotation).as_quat()
        self.fk.setText('FK 目标末端 / m：X {:+.4f}  Y {:+.4f}  Z {:+.4f}\n'
                        '姿态四元数 xyzw：{:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}'.format(*tip, *quat))

    def can_send(self):
        if not self.ready():
            self.events.appendPlainText('未发送：等待手动模式起飞完成及新鲜反馈。')
            return False
        return True

    def send_body(self):
        if not self.can_send():
            return
        msg = PoseStamped(); msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [v.value() for v in self.body[:3]]
        quat = Rotation.from_euler('z', self.body[3].value()).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quat)
        self.body_pub.publish(msg)
        self.events.appendPlainText('已发送机体目标，等待控制器确认。')

    def send_arm(self):
        if not self.can_send():
            return
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(NAMES); msg.position = [v.value() for v in self.arm]
        self.arm_pub.publish(msg)
        self.events.appendPlainText('已发送七关节目标，等待控制器确认。')

    def pump(self):
        if not rclpy.ok():
            QtWidgets.QApplication.quit(); return
        rclpy.spin_once(self, timeout_sec=0.)
        ready = bool(self.ready())
        self.editors.setEnabled(ready)
        if not ready:
            # A new flight or recovered feedback must start from controller
            # targets, not from the previous session's unsent editor values.
            self.initialized = False
        self.status.setText('可控制 · 松开滑条发送目标' if ready else '控制未启用 · 等待起飞 / 手动模式 / 新鲜 OFFBOARD 反馈')


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    node = ManualSliders(app)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    try:
        app.exec_()
    finally:
        node.timer.stop(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
