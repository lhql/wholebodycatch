#!/usr/bin/python3
"""RViz position-only EE goal and measured whole-body visualization."""
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from astra_wall_control.goals import validate_goal
from astra_wall_control.visualization import FlightVisualization
from astra_wall_control.rviz_controls import RvizStateControls


class AirGoal(Node):
    def __init__(self):
        super().__init__('air_goal')
        share = Path(get_package_share_directory('astra_wall_control'))
        self.declare_parameter('scene_file', str(share/'worlds/air.world'))
        self.declare_parameter('history_points', 1500)
        self.declare_parameter('enable_goal_input', True)
        self.renderer = FlightVisualization(share, self.get_parameter('scene_file').value,
                                            max(10, min(10000, self.get_parameter('history_points').value)))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.layers = {name: self.create_publisher(MarkerArray, '/aerial_visualization/'+name, qos)
                       for name in self.renderer.layers}
        self.paths = {name: self.create_publisher(RosPath, '/aerial_visualization/'+name, qos)
                      for name in ('body_path', 'ee_path', 'reference_path', 'planned_path')}
        self.declare_parameter('goal_kind', 'ee')
        self.declare_parameter('control_mode', 'wholebody')
        self.control_mode = self.get_parameter('control_mode').value
        self.goal_kind = self.get_parameter('goal_kind').value
        self.goal_key = 'target' if self.goal_kind == 'ee' else 'body_target'
        self.pub = self.create_publisher(PoseStamped, '/ee_goal' if self.goal_kind == 'ee' else '/body_goal', 10)
        self.visual = self.create_publisher(MarkerArray, '/air_test/markers', 10)
        self.server = InteractiveMarkerServer(self, '/air_test/goal')
        self.tf = TransformBroadcaster(self)
        self.last_feedback = time.monotonic()
        self.row = None
        self.received = -1e9
        marker = InteractiveMarker()
        marker.header.frame_id = 'map'
        marker.name = 'ee_goal'
        marker.description = ('EE' if self.goal_kind == 'ee' else 'Body')+' goal: drag XYZ, release to send'
        marker.scale = .5
        marker.pose.position.x = 2.
        marker.pose.position.z = 1.4
        marker.pose.orientation.w = 1.
        ball = Marker(type=Marker.SPHERE)
        ball.pose.orientation.w = 1.
        ball.scale.x = ball.scale.y = ball.scale.z = .09
        ball.color.r = 1.; ball.color.a = 1.
        visible = InteractiveMarkerControl(always_visible=True)
        visible.markers.append(ball)
        marker.controls.append(visible)
        for name, quaternion in [('x', (0., 0., 0., 1.)),
                                 ('y', (0., 0., .70710678, .70710678)),
                                 ('z', (0., -.70710678, 0., .70710678))]:
            control = InteractiveMarkerControl(name='move_'+name, interaction_mode=InteractiveMarkerControl.MOVE_AXIS)
            control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = quaternion
            marker.controls.append(control)
        if self.control_mode == 'manual':
            control = InteractiveMarkerControl(name='body_yaw', interaction_mode=InteractiveMarkerControl.ROTATE_AXIS)
            control.orientation.y = -.70710678; control.orientation.w = .70710678; marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self.feedback)
        self.state_controls = RvizStateControls(self, self.server, self.control_mode)
        self.server.applyChanges()
        self.create_subscription(String, '/wall_control/telemetry', self.telemetry, 10)
        self.create_subscription(String, '/wall_control/command_status',
                                 lambda msg: setattr(self.renderer, 'command_status', msg.data), 10)
        self.create_timer(.2, self.draw, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def destroy_node(self):
        self.server.shutdown()
        return super().destroy_node()

    def telemetry(self, msg):
        self.row = json.loads(msg.data)
        self.renderer.accept(self.row)
        self.received = time.monotonic()

    def feedback(self, msg):
        self.last_feedback = time.monotonic()
        if self.control_mode == 'manual' and msg.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            p, q = msg.pose.position, msg.pose.orientation
            values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            if np.all(np.isfinite(values)) and np.linalg.norm(values[3:]) > .5:
                self.renderer.preview_body = values
        if msg.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        self.renderer.preview_body = None
        try:
            if not self.get_parameter('enable_goal_input').value:
                raise ValueError('Replay preview: flight goal input disabled')
            validate_goal(msg.header.frame_id, [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            if self.row is None or time.monotonic()-self.received > 1. or self.row['phase'] not in ('APPROACH', 'HOLD'):
                raise ValueError('Wait for fresh APPROACH/HOLD telemetry')
        except ValueError as exc:
            self.renderer.command_status = 'RViz goal REJECTED: '+str(exc)
            self.get_logger().warning(str(exc))
            self.sync_goal()
            return
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = msg.pose
        if self.control_mode != 'manual':
            goal.pose.orientation.x = goal.pose.orientation.y = goal.pose.orientation.z = 0.
            goal.pose.orientation.w = 1.
        self.pub.publish(goal)

    def sync_goal(self):
        if self.row is None:
            return
        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = self.row[self.goal_key]
        q = Rotation.from_euler('z', self.row.get('body_target_yaw', 0.) if self.control_mode == 'manual' else 0.).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
        self.server.setPose('ee_goal', pose)
        self.server.applyChanges()

    def draw(self):
        stale = self.row is not None and time.monotonic()-self.received > 1.
        if time.monotonic()-self.last_feedback > 1.:
            self.sync_goal()
        layers = self.renderer.build(self.row, stale=stale, goal_kind=self.goal_kind)
        self.state_controls.update(self.row, stale)
        all_markers = []
        for name, markers in layers.items():
            for marker in markers:
                marker.header.stamp = self.get_clock().now().to_msg()
            self.layers[name].publish(MarkerArray(markers=markers))
            all_markers.extend(markers if not all_markers else markers[1:])
        self.visual.publish(MarkerArray(markers=all_markers))
        if self.row is not None and not stale:
            transforms = []
            for name, T in self.renderer.geometry.model.frames.items():
                transform = TransformStamped(); transform.header.frame_id = 'map'
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.child_frame_id = 'aerial/'+name
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, T[:3, 3])
                quat = Rotation.from_matrix(T[:3, :3]).as_quat()
                transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, quat)
                transforms.append(transform)
            self.tf.sendTransform(transforms)
        paths = dict(body_path=self.renderer.body_trace, ee_path=self.renderer.tip_trace,
                     reference_path=self.renderer.reference_trace, planned_path=[])
        if self.row is not None and self.row.get('corridor'):
            paths['planned_path'] = self.row['corridor'].get('smooth_path', [self.renderer.route_start]+self.row['corridor']['route'])
        for name, points in paths.items():
            path = RosPath(); path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            for point in points:
                pose = PoseStamped(); pose.header = path.header; pose.pose.orientation.w = 1.
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
                path.poses.append(pose)
            self.paths[name].publish(path)


def main():
    rclpy.init()
    node = AirGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
