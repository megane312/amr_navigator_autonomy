#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from om_msgs.msg import Query, Response, State
from sensor_msgs.msg import Joy

from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters


class JoyIdShareDrive(Node):
    def __init__(self) -> None:
        super().__init__('joy_idshare_drive')

        self.declare_parameter('topic_id', 1)
        self.declare_parameter('global_id', 10)
        self.declare_parameter('axis_num', 2)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('forward_axis', 1)
        self.declare_parameter('turn_axis', 0)
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('max_forward_rpm', 360.0)
        self.declare_parameter('max_turn_rpm', 180.0)
        self.declare_parameter('speed_scale', 2.0)
        self.declare_parameter('publish_period_sec', 0.05)

        self._topic_id = int(self.get_parameter('topic_id').value)
        self._global_id = int(self.get_parameter('global_id').value)
        self._axis_num = int(self.get_parameter('axis_num').value)
        self._joy_topic = str(self.get_parameter('joy_topic').value)
        self._forward_axis = int(self.get_parameter('forward_axis').value)
        self._turn_axis = int(self.get_parameter('turn_axis').value)
        self._deadman_button = int(self.get_parameter('deadman_button').value)
        self._max_forward_rpm = float(self.get_parameter('max_forward_rpm').value)
        self._max_turn_rpm = float(self.get_parameter('max_turn_rpm').value)
        self._speed_scale = float(self.get_parameter('speed_scale').value)
        self._publish_period_sec = float(self.get_parameter('publish_period_sec').value)

        self._query_pub = self.create_publisher(Query, f'om_query{self._topic_id}', 10)
        self._response_sub = self.create_subscription(
            Response, f'om_response{self._topic_id}', self._response_callback, 10
        )
        self._state_sub = self.create_subscription(
            State, f'om_state{self._topic_id}', self._state_callback, 10
        )
        self._joy_sub = self.create_subscription(Joy, self._joy_topic, self._joy_callback, 10)

        self._set_param_client = self.create_client(SetParameters, '/om_node/set_parameters')
        self._joy_msg = None
        self._state_driver = 0
        self._initialized = False
        self._last_drive = (None, None)
        self._response_logged = False

        self.create_timer(self._publish_period_sec, self._timer_callback)

        self.get_logger().info(
            'listening to %s, publishing om_query%d, global_id=%d, axis_num=%d'
            % (self._joy_topic, self._topic_id, self._global_id, self._axis_num)
        )

    def _state_callback(self, msg: State) -> None:
        self._state_driver = msg.state_driver

    def _joy_callback(self, msg: Joy) -> None:
        self._joy_msg = msg

    def _response_callback(self, msg: Response) -> None:
        if msg.func_code not in (0x03, 0x17):
            return
        if self._response_logged:
            return
        self.get_logger().info(
            'response slave_id=%s func_code=0x%02x data[0:8]=%s'
            % (msg.slave_id, msg.func_code & 0xFF, list(msg.data[:8]))
        )
        self._response_logged = True

    def bootstrap(self) -> None:
        if not self._initialized:
            self._enable_drives()
            self._initialized = True

    def _set_remote_global_id(self, value: int) -> None:
        if not self._set_param_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Wait for /om_node/set_parameters timed out')

        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = 'global_id'
        parameter.value.type = ParameterType.PARAMETER_INTEGER
        parameter.value.integer_value = int(value)
        request.parameters = [parameter]

        future = self._set_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            raise RuntimeError('Failed to set om_node/global_id')

    def _publish_query(self, query: Query) -> None:
        self._query_pub.publish(query)
        # small delay to avoid overloading the driver serial thread
        time.sleep(0.02)

    def _write_single(self, slave_id: int, write_addr: int, value: int) -> None:
        query = Query()
        query.slave_id = slave_id
        query.func_code = 1
        query.write_addr = write_addr
        query.write_num = 1
        query.data[0] = value
        self._publish_query(query)

    def _set_share_data(self) -> None:
        for slave_id in (1, 2):
            query = Query()
            query.slave_id = slave_id
            query.func_code = 1
            query.write_addr = 0x0980
            query.write_num = 3
            query.data[0] = self._global_id
            query.data[1] = self._axis_num
            query.data[2] = slave_id
            self._publish_query(query)

            query = Query()
            query.slave_id = slave_id
            query.func_code = 1
            query.write_addr = 0x0990
            query.write_num = 24
            query.data[0] = 45
            query.data[1] = 46
            query.data[2] = 47
            query.data[3] = 48
            query.data[4] = 49
            query.data[5] = 50
            query.data[6] = 102
            query.data[7] = 103
            query.data[8] = 0
            query.data[9] = 0
            query.data[10] = 0
            query.data[11] = 0
            query.data[12] = 45
            query.data[13] = 46
            query.data[14] = 47
            query.data[15] = 48
            query.data[16] = 49
            query.data[17] = 51
            query.data[18] = 0
            query.data[19] = 0
            query.data[20] = 0
            query.data[21] = 0
            query.data[22] = 0
            query.data[23] = 0
            self._publish_query(query)

    def _enable_drives(self) -> None:
        self._set_remote_global_id(-1)
        self._write_single(1, 124, 1)
        self._write_single(2, 124, 1)
        self._set_share_data()
        self._set_remote_global_id(self._global_id)
        self.get_logger().info('ID Share setup completed')

    def _joy_to_drive(self) -> tuple[int, int]:
        if self._joy_msg is None:
            return 0, 0

        buttons = self._joy_msg.buttons
        axes = self._joy_msg.axes

        deadman = 0
        if 0 <= self._deadman_button < len(buttons):
            deadman = buttons[self._deadman_button]

        if not deadman:
            return 0, 0

        forward = 0.0
        turn = 0.0
        if 0 <= self._forward_axis < len(axes):
            forward = axes[self._forward_axis] * self._max_forward_rpm
        if 0 <= self._turn_axis < len(axes):
            turn = axes[self._turn_axis] * self._max_turn_rpm

        right_rpm = int((forward + turn) * self._speed_scale)
        left_rpm = int((forward - turn) * self._speed_scale)
        return right_rpm, left_rpm

    def _publish_drive(self, right_rpm: int, left_rpm: int) -> None:
        query = Query()
        query.slave_id = self._global_id
        query.func_code = 1
        query.write_addr = 0x0000
        query.write_num = 12

        query.data[0] = 16
        query.data[1] = 1000
        query.data[2] = right_rpm
        query.data[3] = 50000
        query.data[4] = 80000
        query.data[5] = 1

        query.data[6] = 16
        query.data[7] = 1000
        query.data[8] = left_rpm
        query.data[9] = 50000
        query.data[10] = 80000
        query.data[11] = 1

        self._publish_query(query)

    def _timer_callback(self) -> None:
        if not self._initialized:
            return

        if self._state_driver == 1:
            return

        right_rpm, left_rpm = self._joy_to_drive()
        if (right_rpm, left_rpm) != self._last_drive:
            joy_forward = 0.0
            joy_turn = 0.0
            if self._joy_msg is not None:
                axes = self._joy_msg.axes
                if 0 <= self._forward_axis < len(axes):
                    joy_forward = axes[self._forward_axis]
                if 0 <= self._turn_axis < len(axes):
                    joy_turn = axes[self._turn_axis]
            self.get_logger().info(
                'joy drive axes forward=%.3f turn=%.3f max_forward=%.1f max_turn=%.1f scale=%.2f left=%d rpm right=%d rpm'
                % (
                    joy_forward,
                    joy_turn,
                    self._max_forward_rpm,
                    self._max_turn_rpm,
                    self._speed_scale,
                    left_rpm,
                    right_rpm,
                )
            )
            self._last_drive = (right_rpm, left_rpm)

        self._publish_drive(right_rpm, left_rpm)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyIdShareDrive()
    try:
        node.bootstrap()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()