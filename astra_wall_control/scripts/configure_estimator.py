#!/usr/bin/env python3
"""Configure the vision-only aerial manipulator: LIO position, height and yaw.
The my airframe has SYS_HAS_MAG=0, so EV yaw is required for alignment.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import State
from mavros_msgs.srv import ParamSetV2
from rcl_interfaces.msg import ParameterValue

# rcl_interfaces/msg/ParameterValue type constants
PARAM_BOOL = 1
PARAM_INTEGER = 2
PARAM_DOUBLE = 3

DEFAULT_PARAMS = [
    ('EKF2_EV_CTRL', PARAM_INTEGER, 11),
    ('EKF2_HGT_REF', PARAM_INTEGER, 3),
    ('EKF2_GPS_CTRL', PARAM_INTEGER, 0),
    ('EKF2_EV_DELAY', PARAM_DOUBLE, 0.0),
    ('EKF2_EV_POS_X', PARAM_DOUBLE, 0.0),
    ('EKF2_EV_POS_Y', PARAM_DOUBLE, 0.0),
    ('EKF2_EV_POS_Z', PARAM_DOUBLE, 0.0),
]


class Ekf2EvConfig(Node):
    def __init__(self):
        super().__init__('ekf2_ev_config')

        self.declare_parameter(
            'service_names', '/mavros/mavros/set,/mavros/param/set')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('connect_timeout', 60.0)

        self._service_names = [
            name.strip()
            for name in str(self.get_parameter('service_names').value).split(',')
            if name.strip()
        ]
        state_topic = self.get_parameter('state_topic').value
        self._connect_timeout = float(self.get_parameter('connect_timeout').value)

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_sub = self.create_subscription(
            State, state_topic, self._on_state, qos)

        self._clients = {
            name: self.create_client(ParamSetV2, name)
            for name in self._service_names
        }
        self._client = None
        self._pending = []
        self._future = None
        self._finished = False
        self._connected = False
        self._start_time = self.get_clock().now()

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            'waiting for MAVROS connection before writing EKF2 vision parameters')

    def _on_state(self, msg):
        if msg.connected and not self._connected:
            self._connected = True
            self.get_logger().info('flight controller connected')

    def _tick(self):
        if self._future is not None:
            if self._future.done():
                try:
                    result = self._future.result()
                    param_id = self._pending[0][0] if self._pending else '?'
                    if result is not None and getattr(result, 'success', True):
                        self.get_logger().info(f'{param_id} set')
                    else:
                        self.get_logger().warning(f'{param_id} was rejected')
                except Exception as exc:  # noqa: BLE001 - report and continue
                    self.get_logger().error(f'parameter write failed: {exc}')
                if self._pending:
                    self._pending.pop(0)
                self._future = None
            return

        if self._finished:
            return

        if not self._connected:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
            if elapsed > self._connect_timeout:
                self.get_logger().error(
                    'timed out waiting for /mavros/state to report a connection')
                self._finished = True
            return

        if self._client is None:
            self._client = self._select_client()
            if self._client is None:
                return
            self._pending = list(DEFAULT_PARAMS)
            self.get_logger().info(
                f'writing EKF2 vision parameters through '
                f'{self._client.srv_name}')

        if not self._pending:
            self.get_logger().info('EKF2 external vision parameters applied')
            self._finished = True
            return

        param_id, value_type, value = self._pending[0]
        request = ParamSetV2.Request()
        request.force_set = False
        request.param_id = param_id
        parameter_value = ParameterValue()
        parameter_value.type = value_type
        if value_type == PARAM_INTEGER:
            parameter_value.integer_value = int(value)
        elif value_type == PARAM_DOUBLE:
            parameter_value.double_value = float(value)
        else:
            parameter_value.bool_value = bool(value)
        request.value = parameter_value
        self._future = self._client.call_async(request)

    @property
    def finished(self):
        return self._finished

    def _select_client(self):
        for name, client in self._clients.items():
            if client.wait_for_service(timeout_sec=0.2):
                return client
            self.get_logger().warning(f'service {name} not available yet')
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Ekf2EvConfig()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
