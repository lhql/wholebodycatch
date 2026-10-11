"""Exercise RViz drag-to-joint-state math with real messages, without a DDS participant."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import time
import numpy as np
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerFeedback
from astra_wall_control.visualization import FlightVisualization
from astra_wall_control.rviz_controls import RvizStateControls
from astra_wall_control.geometry import NAMES
from astra_wall_control.body_motion import BoundedBodyMotion
from geometry_msgs.msg import PoseStamped
from check_ros_goals import load

ROOT = Path(__file__).resolve().parents[1]


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(deepcopy(msg))


class Server:
    def __init__(self):
        marker = InteractiveMarker(); marker.name = 'ee_goal'
        self.markers = {'ee_goal': marker}
    def get(self, name): return self.markers.get(name)
    def insert(self, marker, **kwargs): self.markers[marker.name] = marker
    def applyChanges(self): pass


def main():
    renderer = FlightVisualization(ROOT, ROOT/'worlds/air.world')
    q = np.array([0., -.05, .1, .05, 0., .01, .01])
    row = dict(t=1., phase='HOLD', control_mode='manual', body=[.5, .2, 1.6], body_target=[.5, .2, 1.6],
               body_orientation=Rotation.from_euler('xyz', [.05, -.03, .3]).as_quat().tolist(),
               tip=[.9, .2, 1.4], target=[.9, .2, 1.4], ee_error=0., q=q.tolist(), joint_goal=q.tolist())
    publications = {}
    def publisher(kind, topic, depth):
        publications[topic] = Publisher(); return publications[topic]
    enabled = SimpleNamespace(value=True)
    node = SimpleNamespace(row=row, renderer=renderer, received=time.monotonic(),
                           create_publisher=publisher, get_parameter=lambda name: enabled,
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())))
    server = Server(); controls = RvizStateControls(node, server, 'manual')
    for index in range(7):
        controls.select(index); renderer.build(row, goal_kind='body'); controls.update(row, False)
        feedback = InteractiveMarkerFeedback(); feedback.header.frame_id = 'map'
        feedback.pose = deepcopy(server.markers['arm_joint'].pose)
        feedback.event_type = feedback.MOUSE_DOWN; controls.joint_feedback(feedback)
        origin, axis, R, initial, kind = controls.baseline
        delta = .002 if kind == 'prismatic' else .03
        if kind == 'prismatic':
            moved = origin+axis*delta
            feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z = map(float, moved)
        else:
            quat = Rotation.from_matrix(R@Rotation.from_rotvec([delta, 0., 0.]).as_matrix()).as_quat()
            feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w = map(float, quat)
        previous = len(publications['/manual/arm_goal'].messages)
        feedback.event_type = feedback.POSE_UPDATE; controls.joint_feedback(feedback)
        assert len(publications['/manual/arm_goal'].messages) == previous  # preview only
        np.testing.assert_allclose(renderer.preview_q[index], q[index]+delta)
        feedback.event_type = feedback.MOUSE_UP; controls.joint_feedback(feedback)
        command = publications['/manual/arm_goal'].messages[-1]
        assert command.name == NAMES
        expected = q.copy(); expected[index] += delta
        np.testing.assert_allclose(command.position, expected)
        assert not controls.dragging
    enabled.value = False
    count = len(publications['/manual/arm_goal'].messages)
    controls.joint_feedback(feedback)
    assert len(publications['/manual/arm_goal'].messages) == count
    # Manual FK preview must not overwrite measured transforms used for TF.
    row['joint_goal'][2] = .13
    layers = renderer.build(row, goal_kind='body')
    predicted = next(m for m in layers['goals'] if m.ns == 'fk_predicted_tip')
    expected = renderer.preview_geometry.model.evaluate(np.array(row['body_target']), np.eye(3), np.array(row['joint_goal']))[0]
    np.testing.assert_allclose([predicted.pose.position.x, predicted.pose.position.y, predicted.pose.position.z], expected)
    np.testing.assert_allclose(renderer.geometry.model.frames['aircraft_base'][:3, 3], row['body'])
    # Bounded body references leave the end-effector goal untouched.
    events = []
    controller = SimpleNamespace(ready_for_goal=lambda: None, motion_started=False, phase='HOLD',
        motion=BoundedBodyMotion(), body_center=np.array([.6, 0., 1.6]), body_target=np.array([.6, 0., 1.6]),
        yaw_ref=0., target=np.array([1., 0., 1.4]), command_status=events.append)
    goal = PoseStamped(); goal.header.frame_id = 'map'; goal.pose.orientation.w = 1.
    goal.pose.position.x = .63; goal.pose.position.z = 1.6
    callback = load('wall_control').WallControl.on_wholebody_body_goal
    callback(controller, goal)
    np.testing.assert_allclose(controller.motion.offset, [.03, 0., 0., 0.])
    np.testing.assert_equal(controller.target, [1., 0., 1.4])
    goal.pose.position.x = 5.; callback(controller, goal)
    np.testing.assert_allclose(controller.motion.offset, [.03, 0., 0., 0.])
    assert 'REJECTED' in events[-1]
    print('Seven RViz joint drags, FK-only previews, replay input lock and bounded WBC references: PASS')


if __name__ == '__main__': main()
