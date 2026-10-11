"""Mode ownership, rejection and timeout checks using real ROS message types."""
import time
import json
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from check_ros_goals import load
from astra_wall_control.geometry import NAMES


class Capture:
    def publish(self, message): self.row = json.loads(message.data)


def check_wholebody_publication():
    rclpy.init(args=['--ros-args', '-p', 'control_mode:=wholebody', '-p', 'interactive_goal:=true',
                    '-p', 'hold_body_motion:=true'])
    node = load('wall_control').WallControl()
    capture = Capture(); node.report = capture
    try:
        node.phase = 'HOLD'; node.q = node.home.copy()
        node.pose = np.array([0., 0., 0., 0., 0., 0., 1.])
        node.measured_rotation = np.eye(3)
        for error, enabled, paused in [(0., True, False), (.1, True, True), (0., False, True)]:
            node.motion_enabled = enabled
            p = node.body_center+np.array([error, 0., 0.])
            node.wholebody_step(p, np.eye(3), .02)
            tip, tool, _ = node.model.evaluate(p, np.eye(3), node.q)
            node.publish_control(node.get_clock().now(), node.last_report+1., 1., p, tip, tool, 0.)
            assert capture.row['wholebody']['started']
            assert capture.row['wholebody']['paused'] is paused
        print('wholebody: running/error-paused/user-paused telemetry publication PASS')
    finally:
        node.destroy_node(); rclpy.shutdown()


def check(mode):
    rclpy.init(args=['--ros-args', '-p', 'control_mode:='+mode, '-p', 'interactive_goal:=true',
                    '-p', 'goal_kind:=body'])
    node = load('wall_control').WallControl()
    ui = load('air_goal').AirGoal()
    try:
        goal = PoseStamped(); goal.header.frame_id = 'map'; goal.pose.orientation.w = 1.
        goal.pose.position.x = 2.7; goal.pose.position.z = 1.6
        initial = node.body_target.copy()
        node.on_body_goal(goal)
        np.testing.assert_equal(node.body_target, initial)
        node.phase = 'HOLD'; node.q = node.home.copy()
        node.measured_position = np.array([0., 0., 1.6])
        node.on_body_goal(goal)
        np.testing.assert_equal(node.body_target, [2.7, 0., 1.6])
        if mode == 'manual':
            msg = JointState(); msg.name = NAMES; msg.position = node.home.tolist()
            msg.position[0] = .1; node.on_arm_goal(msg)
            assert node.arm_goal[0] == .1
            msg.position[0] = 99.; node.on_arm_goal(msg)
            assert node.arm_goal[0] == .1
            velocity = TwistStamped(); velocity.header.frame_id = 'map'
            velocity.header.stamp = node.get_clock().now().to_msg(); velocity.twist.linear.x = 10.
            node.on_velocity(velocity); assert node.manual_velocity[0] == .18
            node.independent_step(node.measured_position, .02)
            assert node.p_ref[0] > 0.
            node.velocity_received = time.monotonic()-1.
            node.independent_step(node.measured_position, .02)
            np.testing.assert_equal(node.p_ref, node.measured_position)
            velocity.header.frame_id = 'base_link'; velocity.twist.linear.x = -.1
            node.on_velocity(velocity); assert node.manual_velocity[0] == .18
            velocity.header.frame_id = 'map'; velocity.header.stamp.sec -= 2
            node.on_velocity(velocity); assert node.manual_velocity[0] == .18
        else:
            assert len(node.corridor.waypoints) == 3
            node.independent_step(node.measured_position, .02)
            goal.pose.position.x = 1.4
            node.on_body_goal(goal)
            np.testing.assert_equal(node.body_target, [2.7, 0., 1.6])
            np.testing.assert_equal(node.arm_goal, node.home)
        node.abort = True; goal.pose.position.x = 0.
        previous = node.body_target.copy(); node.on_body_goal(goal)
        np.testing.assert_equal(node.body_target, previous)
        # Visualization runs for both the body goal and corridor schema.
        ui.row = dict(target=node.target.tolist(), body_target=node.body_target.tolist(),
                      body=[0., 0., 1.6], tip=[.4, 0., 1.4], phase='HOLD', ee_error=0., body_error=0.)
        if node.corridor:
            ui.row['corridor'] = dict(cells=node.corridor.cells.tolist(),
                                     route=[p.tolist() for p in node.corridor.waypoints],
                                     index=0, radius=node.corridor.radius)
        ui.received = time.monotonic(); ui.draw(); ui.sync_goal()
        node.pose = np.array([0., 0., 0., 0., 0., 0., 1.]); node.measured_rotation = np.eye(3)
        node.report = Capture()
        tip, tool, _ = node.model.evaluate(node.measured_position, np.eye(3), node.q)
        node.publish_control(node.get_clock().now(), node.last_report+1., 1., node.measured_position, tip, tool, 0.)
        assert node.report.row['control_mode'] == mode
        if node.corridor: assert isinstance(node.report.row['corridor']['trajectory_paused'], bool)
        print(mode+': mode callbacks, gates, references and visualization PASS')
    finally:
        ui.destroy_node(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    check('manual')
    check('corridor')
    check_wholebody_publication()
