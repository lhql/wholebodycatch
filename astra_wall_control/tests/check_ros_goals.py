"""ROS interface smoke check; run explicitly with the ROS/workspace environment sourced."""
import importlib.util
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerFeedback


def load(name):
    path = Path(__file__).resolve().parents[1] / 'scripts' / (name+'.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    rclpy.init(args=['--ros-args', '-p', 'interactive_goal:=true'])
    controller = load('wall_control').WallControl()
    ui = load('air_goal').AirGoal()
    try:
        assert controller.interactive and not controller.control.wall_constraint
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 1.
        goal.pose.position.y = .5
        goal.pose.position.z = 1.5
        initial = controller.target.copy()
        controller.on_goal(goal)
        np.testing.assert_equal(controller.target, initial)  # reject before takeoff
        controller.phase = 'HOLD'
        controller.on_goal(goal)
        np.testing.assert_equal(controller.target, [1., .5, 1.5])
        np.testing.assert_allclose(controller.body_target, controller.target-controller.tip_offset)
        assert controller.phase == 'APPROACH'
        goal.header.frame_id = 'odom'
        controller.on_goal(goal)
        np.testing.assert_equal(controller.target, [1., .5, 1.5])
        goal.header.frame_id = 'map'
        controller.on_goal_2d(goal)
        assert controller.target[2] == 1.4
        controller.abort = True
        goal.pose.position.x = 2.
        controller.on_goal(goal)
        assert controller.target[0] == 1.
        import time
        ui.row = dict(target=[1., .5, 1.4], body_target=[.6, .5, 1.6], body=[.6, .5, 1.6],
                      tip=[1., .5, 1.4], phase='HOLD', ee_error=0., body_error=0.)
        ui.received = time.monotonic()
        ui.draw()
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = 'map'
        feedback.pose = goal.pose
        feedback.event_type = InteractiveMarkerFeedback.MOUSE_UP
        ui.feedback(feedback)
        ui.sync_goal()
        print('ROS goal callbacks, rejection gates, marker creation/publication: PASS')
    finally:
        ui.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
