#!/usr/bin/python3
"""Live, read-only flight dashboard paired with the RViz geometry view."""
from collections import deque
import json
import signal
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from PyQt5 import QtCore, QtGui, QtWidgets
from astra_wall_control.geometry import NAMES


class ErrorPlot(QtWidgets.QWidget):
    def __init__(self):
        super().__init__(); self.rows = deque(maxlen=300); self.setMinimumHeight(150)

    def add(self, row):
        if self.rows and row['t'] < self.rows[-1][0]: self.rows.clear()
        if self.rows and row['t'] == self.rows[-1][0]: return
        self.rows.append((row['t'], row['body_error'], row['ee_error']))
        while self.rows and row['t']-self.rows[0][0] > 30.:
            self.rows.popleft()
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self); painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#101923'))
        plot = QtCore.QRectF(48., 28., self.width()-65., self.height()-50.)
        painter.setPen(QtGui.QColor('#66788b')); painter.drawRect(plot)
        maximum = max(.1, max((max(row[1:]) for row in self.rows), default=.1))
        painter.drawText(5, 35, '{:.2f}m'.format(maximum)); painter.drawText(14, self.height()-20, '0')
        painter.setPen(QtGui.QColor('#33b8ff')); painter.drawText(52, 18, '机体误差')
        painter.setPen(QtGui.QColor('#50ed91')); painter.drawText(155, 18, '末端误差（估计）')
        if len(self.rows) < 2: return
        span = max(self.rows[-1][0]-self.rows[0][0], .01)
        for k, color in ((1, '#33b8ff'), (2, '#50ed91')):
            path = QtGui.QPainterPath()
            for i, row in enumerate(self.rows):
                x = plot.left()+(row[0]-self.rows[0][0])/span*plot.width()
                y = plot.bottom()-row[k]/maximum*plot.height()
                if i == 0: path.moveTo(x, y)
                else: path.lineTo(x, y)
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 2.)); painter.drawPath(path)
        painter.setPen(QtGui.QColor('#aabbcc'))
        painter.drawText(self.width()-160, self.height()-4, '最近 {:.1f} s 仿真时间'.format(span))


class Dashboard(Node):
    def __init__(self, app):
        super().__init__('flight_dashboard')
        self.row = None; self.received = -1e9; self.joints = None; self.joint_received = -1e9
        self.window = QtWidgets.QWidget(); self.window.setWindowTitle('Astra · 飞行与机械臂监控')
        self.window.resize(575, 900); self.window.move(1290, 30)
        # Keep every joint reachable on smaller displays without compressing
        # the table until the fingers disappear below its viewport.
        content = QtWidgets.QWidget()
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(content)
        outer = QtWidgets.QVBoxLayout(self.window); outer.setContentsMargins(0, 0, 0, 0); outer.addWidget(scroll)
        self.window.setStyleSheet('''
            QWidget { background:#16212d; color:#e4edf7; font-size:13px; }
            QLabel#title { font-size:23px; font-weight:600; color:#61caff; }
            QLabel#phase { font-size:18px; font-weight:600; padding:8px; background:#263849; }
            QGroupBox { border:1px solid #385065; border-radius:5px; margin-top:12px; padding-top:12px; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; color:#91d5ff; }
            QTableWidget { background:#101923; gridline-color:#304458; }
            QHeaderView::section { background:#263849; padding:5px; border:0; }
            QPlainTextEdit { background:#101923; border:1px solid #304458; }
        ''')
        layout = QtWidgets.QVBoxLayout(content)
        title = QtWidgets.QLabel('ASTRA  /  全身任务监控'); title.setObjectName('title'); layout.addWidget(title)
        subtitle = QtWidgets.QLabel('只读监控 · RViz 设置目标 · 键盘操作机械臂/飞行')
        subtitle.setWordWrap(True); layout.addWidget(subtitle)
        self.phase = QtWidgets.QLabel('等待控制器反馈'); self.phase.setObjectName('phase'); layout.addWidget(self.phase)
        self.metrics = QtWidgets.QLabel(); self.metrics.setWordWrap(True); layout.addWidget(self.metrics)
        self.plot = ErrorPlot(); layout.addWidget(self.plot)
        positions = QtWidgets.QGroupBox('位置与目标  ·  map / ENU / m')
        grid = QtWidgets.QGridLayout(positions); self.values = {}
        for i, (key, label) in enumerate([('body', '机体实际'), ('body_target', '机体目标'),
                                         ('tip', '末端实际'), ('target', '末端目标'),
                                         ('body_reference', '飞控设定点')]):
            grid.addWidget(QtWidgets.QLabel(label), i, 0)
            value = QtWidgets.QLabel('—'); value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            grid.addWidget(value, i, 1); self.values[key] = value
        layout.addWidget(positions)
        self.corridor = QtWidgets.QLabel('安全包络与走廊：等待数据'); self.corridor.setWordWrap(True)
        layout.addWidget(self.corridor)
        self.table = QtWidgets.QTableWidget(7, 5)
        self.table.setHorizontalHeaderLabels(['关节/单位', '实际', '目标', '速度', '力矩/力'])
        self.table.verticalHeader().hide(); self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(25)
        self.table.verticalHeader().setMinimumSectionSize(25)
        self.table.setFixedHeight(7*25+34)
        for i, label in enumerate(['yaw rad', 'shoulder rad', 'extension m', 'wrist rad', 'roll rad', 'left m', 'right m']):
            for j in range(5): self.table.setItem(i, j, QtWidgets.QTableWidgetItem(label if j == 0 else '—'))
        layout.addWidget(self.table)
        legend = QtWidgets.QLabel('RViz 图例：蓝色=机体轨迹  绿色=末端轨迹  紫色=设定点\n'
                                 '黄色=规划路线  红色=目标/误差线  橙色=障碍物\n'
                                 '绿色走廊=当前段  琥珀色=下一段  青色=其他走廊')
        legend.setWordWrap(True); layout.addWidget(legend)
        self.events = QtWidgets.QPlainTextEdit(); self.events.setReadOnly(True)
        self.events.setMaximumBlockCount(50); self.events.setMaximumHeight(92)
        self.events.setPlaceholderText('目标接受/拒收、故障原因将在这里显示'); layout.addWidget(self.events)
        footer = QtWidgets.QLabel('以上为控制器估计反馈；物理到达精度须用独立 Gazebo 真值验收。')
        footer.setWordWrap(True); footer.setStyleSheet('color:#e5bd70; font-size:12px;'); layout.addWidget(footer)
        self.create_subscription(String, '/wall_control/telemetry', self.telemetry, 10)
        self.create_subscription(String, '/wall_control/command_status', lambda msg: self.events.appendPlainText(msg.data), 10)
        self.create_subscription(JointState, '/my/arm/joint_states', self.on_joints, rclpy.qos.qos_profile_sensor_data)
        self.timer = QtCore.QTimer(); self.timer.timeout.connect(self.pump); self.timer.start(40)
        screen = app.primaryScreen().availableGeometry()
        self.window.resize(min(575, screen.width()), min(960, screen.height()))
        self.window.move(max(screen.left(), min(1290, screen.right()-self.window.width()+1)), screen.top())
        self.window.show()

    def on_joints(self, msg):
        self.joints = msg; self.joint_received = time.monotonic()

    def telemetry(self, msg):
        self.row = json.loads(msg.data); self.received = time.monotonic(); self.plot.add(self.row)

    def pump(self):
        if not rclpy.ok():
            QtWidgets.QApplication.quit(); return
        rclpy.spin_once(self, timeout_sec=0.)
        if self.row is None: return
        row = self.row; stale = time.monotonic()-self.received > 1.
        mode = {'manual': '空中独立控制', 'corridor': '全身安全走廊', 'wholebody': '末端全身定点'}.get(row.get('control_mode'), '全身控制')
        self.phase.setText('{}  |  {}'.format(mode, '反馈失效 STALE' if stale else row['phase']))
        self.phase.setStyleSheet('color:'+('#ff796e' if stale else '#70edbd'))
        self.metrics.setText('机体 {:.3f} m  ·  末端 {:.3f} m  ·  求解 {:.2f} ms\n'
                             '{}  ·  {}  ·  t = {:.1f} s'.format(row.get('body_error', 0.), row['ee_error'],
                             row.get('solve_ms', 0.), row.get('mode', '—'), '已解锁' if row.get('armed') else '未解锁', row.get('t', 0.)))
        for key, label in self.values.items():
            xyz = row.get(key)
            if xyz is not None: label.setText('x {:+.3f}    y {:+.3f}    z {:+.3f}'.format(*xyz))
        c = row.get('corridor')
        if c:
            route_cells = c.get('route_cells', list(range(len(c['cells']))))
            active = route_cells[min(c['index'], len(route_cells)-1)]
            low, high = np.asarray(c['cells'][active]); p = np.asarray(row['body'])
            clearance = min(np.min(p-low), np.min(high-p))
            self.corridor.setText('包络 {:.3f} m + 余量 {:.2f} m\n走廊 C{}  ·  路段 {}/{}  ·  中心剩余净空 {:.3f} m'.format(
                c['radius'], c.get('margin', .1), active, c['index']+1, len(c['route']), clearance))
            self.corridor.setStyleSheet('color:'+('#ff796e' if clearance < 0 else '#70edbd'))
            if 'trajectory_time' in c:
                self.corridor.setText(self.corridor.text()+'\n最小 jerk 轨迹 {:.1f}/{:.1f} s · {}'.format(
                    c['trajectory_time'], c['trajectory_duration'], '等待跟踪' if c['trajectory_paused'] else '执行中'))
        elif 'wholebody' in row:
            w = row['wholebody']; bounds = w['bounds']
            self.corridor.setText('末端世界位置固定 · 机体范围 ±({:.3f}, {:.3f}, {:.3f}) m\n'
                                 '偏航 ±{:.2f} rad · 摆动 {} · 周期 {:.1f} s'.format(*bounds[:3], bounds[3],
                                 '暂停/等待末端恢复' if w['paused'] else '执行中' if w['started'] else '等待末端到点', w['period']))
        else:
            self.corridor.setText('全身包络仅作尺寸显示；当前模式没有走廊避障约束。')
        for i, name in enumerate(NAMES):
            q = row.get('q', []); command = row.get('joint_goal', row.get('joint_target', []))
            values = [q[i] if i < len(q) else None, command[i] if i < len(command) else None, None, None]
            if self.joints is not None and time.monotonic()-self.joint_received < .6 and name in self.joints.name:
                j = self.joints.name.index(name)
                values[2] = self.joints.velocity[j] if j < len(self.joints.velocity) else None
                values[3] = self.joints.effort[j] if j < len(self.joints.effort) else None
            for j, value in enumerate(values):
                self.table.item(i, j+1).setText('—' if value is None else '{:.3f}'.format(value))


def main():
    rclpy.init(); app = QtWidgets.QApplication(sys.argv); node = Dashboard(app)
    signal.signal(signal.SIGINT, lambda signum, frame: app.quit())
    signal.signal(signal.SIGTERM, lambda signum, frame: app.quit())
    try: app.exec_()
    finally:
        node.timer.stop(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
