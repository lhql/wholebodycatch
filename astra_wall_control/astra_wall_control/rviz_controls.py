"""RViz state editors: one selected joint gizmo, and bounded WBC body reference."""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler
from .geometry import NAMES


def axis_rotation(axis):
    axis = np.asarray(axis, float); axis /= np.linalg.norm(axis)
    cross = np.cross([1., 0., 0.], axis); norm = np.linalg.norm(cross)
    if norm < 1e-9:
        return Rotation.from_rotvec([0., 0., np.pi if axis[0] < 0 else 0.]).as_matrix()
    return Rotation.from_rotvec(cross/norm*np.arctan2(norm, axis[0])).as_matrix()


class RvizStateControls:
    def __init__(self, node, server, mode):
        self.node, self.server, self.mode = node, server, mode
        self.selected = 0; self.dragging = False; self.baseline = None
        self.menu = MenuHandler()
        self.events = node.create_publisher(String, '/wall_control/command_status', 10)
        self.arm_pub = node.create_publisher(JointState, '/manual/arm_goal', 10)
        self.body_pub = node.create_publisher(PoseStamped, '/wholebody/body_goal', 10)
        self.motion_pub = node.create_publisher(Bool, '/wholebody/motion_enabled', 10)
        if mode == 'manual':
            for index, name in enumerate(NAMES):
                self.menu.insert('{} {}'.format(index+1, name), callback=lambda msg, i=index: self.select(i))
        elif mode == 'wholebody':
            self.menu.insert('Pause / resume bounded body motion', callback=self.toggle_motion)
        if mode in ('manual', 'wholebody'): self.menu.apply(server, 'ee_goal')

    def report(self, text):
        self.node.renderer.command_status = text; self.events.publish(String(data=text))

    def ready(self):
        if not self.node.get_parameter('enable_goal_input').value:
            raise ValueError('Replay preview: flight goal input disabled')
        row = self.node.row
        if row is None or time.monotonic()-self.node.received > 1. or row['phase'] not in ('APPROACH', 'HOLD'):
            raise ValueError('Wait for fresh airborne telemetry')

    def select(self, index):
        self.selected = index; self.dragging = False; self.node.renderer.preview_q = None
        self.report('RViz selected '+NAMES[index])

    def toggle_motion(self, msg):
        try: self.ready()
        except ValueError as exc: self.report(str(exc)); return
        enabled = self.node.row.get('wholebody', {}).get('enabled', False)
        self.motion_pub.publish(Bool(data=not enabled))

    @staticmethod
    def visual(marker, color):
        ball = Marker(type=Marker.SPHERE); ball.pose.orientation.w = 1.
        ball.scale.x = ball.scale.y = ball.scale.z = .04
        ball.color.r, ball.color.g, ball.color.b = color; ball.color.a = 1.
        visible = InteractiveMarkerControl(always_visible=True); visible.markers.append(ball)
        marker.controls.append(visible)

    def update(self, row, stale):
        if row is None: return
        if stale: self.dragging = False
        if self.dragging: return
        if self.mode == 'manual':
            model = self.node.renderer.geometry.model
            origin, axis, kind = model.axes[self.selected]
            R = axis_rotation(axis)
            q = np.array(row.get('joint_goal', row.get('q', [0., -.05, .1, .05, 0., .01, .01])))
            self.baseline = (origin.copy(), axis.copy(), R.copy(), q.copy(), kind)
            marker = InteractiveMarker(); marker.header.frame_id = 'map'; marker.name = 'arm_joint'
            unit = 'm' if kind == 'prismatic' else 'rad'
            marker.description = '{}: {:.3f} {} (right-click to select)'.format(NAMES[self.selected], q[self.selected], unit)
            marker.scale = .32
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, origin)
            quat = Rotation.from_matrix(R).as_quat()
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = map(float, quat)
            self.visual(marker, (1., .6, .1))
            control = InteractiveMarkerControl(name='joint_value')
            control.orientation.w = 1.
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS if kind == 'prismatic' else InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)
            self.server.insert(marker, feedback_callback=self.joint_feedback); self.menu.apply(self.server, marker.name)
        elif self.mode == 'wholebody' and 'wholebody' in row:
            state = row['wholebody']; offset = np.asarray(state['offset'])
            marker = InteractiveMarker(); marker.header.frame_id = 'map'; marker.name = 'wholebody_body'
            marker.description = 'BOUNDED BODY REFERENCE (EE stays fixed)'; marker.scale = .28
            xyz = np.array(state['center'])+offset[:3]
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, xyz)
            quat = Rotation.from_euler('z', offset[3]).as_quat()
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = map(float, quat)
            self.visual(marker, (.8, .3, 1.))
            for name, quat in [('x', (0., 0., 0., 1.)), ('y', (0., 0., .70710678, .70710678)), ('z', (0., -.70710678, 0., .70710678))]:
                control = InteractiveMarkerControl(name='body_'+name, interaction_mode=InteractiveMarkerControl.MOVE_AXIS)
                control.orientation_mode = InteractiveMarkerControl.FIXED
                control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = quat
                marker.controls.append(control)
            control = InteractiveMarkerControl(name='body_yaw', interaction_mode=InteractiveMarkerControl.ROTATE_AXIS)
            control.orientation.y = -.70710678; control.orientation.w = .70710678
            marker.controls.append(control)
            self.server.insert(marker, feedback_callback=self.body_feedback); self.menu.apply(self.server, marker.name)
        self.server.applyChanges()

    def joint_feedback(self, msg):
        if msg.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.dragging = True; return
        if msg.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP): return
        try:
            self.ready()
            if msg.header.frame_id != 'map' or self.baseline is None: raise ValueError('Invalid joint feedback frame')
            origin, axis, R, q, kind = self.baseline; candidate = q.copy()
            if kind == 'prismatic':
                moved = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                delta = float(np.dot(moved-origin, axis))
            else:
                o = msg.pose.orientation; measured = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
                relative = R.T@measured; delta = float(np.arctan2(relative[2, 1], relative[1, 1]))
            candidate[self.selected] += delta
            model = self.node.renderer.geometry.model
            if not np.all(np.isfinite(candidate)) or np.any(candidate < model.lower) or np.any(candidate > model.upper):
                raise ValueError('Joint target exceeds its URDF limit')
            self.node.renderer.preview_q = candidate
            if msg.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                command = JointState(); command.header.stamp = self.node.get_clock().now().to_msg()
                command.name = NAMES; command.position = candidate.tolist(); self.arm_pub.publish(command)
                self.report('Sent RViz joint state; waiting for controller acceptance')
        except ValueError as exc: self.report('Joint state REJECTED: '+str(exc))
        if msg.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.dragging = False; self.node.renderer.preview_q = None

    def body_feedback(self, msg):
        if msg.event_type == InteractiveMarkerFeedback.MOUSE_DOWN: self.dragging = True
        if msg.event_type != InteractiveMarkerFeedback.MOUSE_UP: return
        self.dragging = False
        try: self.ready()
        except ValueError as exc: self.report(str(exc)); return
        goal = PoseStamped(); goal.header = msg.header; goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose = msg.pose; self.body_pub.publish(goal)
