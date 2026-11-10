#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from om_msgs.msg import Response, State
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(half_yaw)
    quat.w = math.cos(half_yaw)
    return quat


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__('om_odom_tf_broadcaster')

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('topic_id', 1)
        self.declare_parameter('expected_slave_id', 1)
        self.declare_parameter('accept_any_response', False)

        self._odom_frame_id = self.get_parameter('odom_frame_id').value
        self._base_frame_id = self.get_parameter('base_frame_id').value
        self._topic_id = int(self.get_parameter('topic_id').value)
        self._expected_slave_id = int(self.get_parameter('expected_slave_id').value)
        self._accept_any_response = bool(self.get_parameter('accept_any_response').value)

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = self._odom_frame_id
        self._odom_msg.child_frame_id = self._base_frame_id

        self._state_driver = 0
        self._rejection_logged = False
        self._invalid_response_logged = False

        self.get_logger().info(
            f'listening to om_response{self._topic_id} / om_state{self._topic_id}, '
            f'expecting slave_id={self._expected_slave_id}, '
            f'publishing /odom as {self._odom_frame_id} -> {self._base_frame_id}, '
            f'accept_any_response={self._accept_any_response}'
        )

        self.create_subscription(Response, f'om_response{self._topic_id}', self._response_callback, 10)
        self.create_subscription(State, f'om_state{self._topic_id}', self._state_callback, 10)

    def _state_callback(self, msg: State) -> None:
        self._state_driver = msg.state_driver

    def _response_callback(self, msg: Response) -> None:
        expected_slave = self._expected_slave_id

        # Temporary verbose logging to help inspect live Response messages
        if self._accept_any_response:
            try:
                self.get_logger().info(
                    'received om_response%s: slave_id=%s func_code=0x%02x data[0:12]=%s'
                    % (self._topic_id, msg.slave_id, msg.func_code & 0xFF, list(msg.data[:12]))
                )
            except Exception:
                # swallow logging errors to avoid crashing the callback
                pass

        # BLV/ID Share系の応答は 0x17 だけでなく 0x03 でも返ることがあるため、
        # どちらも同じ odom 変換の対象として扱う。
        valid_func_codes = (0x03, 0x17)
        if not self._accept_any_response and (
            msg.slave_id != expected_slave or msg.func_code not in valid_func_codes
        ):
            if not self._rejection_logged:
                self.get_logger().info(
                    'ignoring om_response%s: slave_id=%s func_code=0x%02x expected slave_id=%s func_code=0x03/0x17'
                    % (self._topic_id, msg.slave_id, msg.func_code & 0xFF, expected_slave)
                )
                self.get_logger().info(
                    'om_response%s sample data[0:8]=%s'
                    % (self._topic_id, list(msg.data[:8]))
                )
                self._rejection_logged = True
            return

        if not self._response_quality(msg):
            if not self._invalid_response_logged:
                self.get_logger().warning(
                    'ignoring om_response%s: payload is not suitable for odom mapping'
                    % (self._topic_id)
                )
                self._invalid_response_logged = True
            return

        self._invalid_response_logged = False

        self._rejection_logged = False

        stamp = self.get_clock().now().to_msg()
        self._odom_msg.header.stamp = stamp
        self._odom_msg.header.frame_id = self._odom_frame_id
        self._odom_msg.child_frame_id = self._base_frame_id
        self._odom_msg.pose.pose.position.x = float(msg.data[0] / 1000.0)
        self._odom_msg.pose.pose.position.y = float(msg.data[1] / 1000.0)
        self._odom_msg.pose.pose.position.z = 0.0
        self._odom_msg.pose.pose.orientation = quaternion_from_yaw(float(msg.data[2]) / 1000000.0)
        self._odom_msg.twist.twist.linear.x = float(msg.data[5] / 1000.0)
        self._odom_msg.twist.twist.linear.y = float(msg.data[7] / 1000.0)
        self._odom_msg.twist.twist.angular.z = float(msg.data[6] / 1000000.0)

        transform = TransformStamped()
        transform.header = self._odom_msg.header
        transform.child_frame_id = self._odom_msg.child_frame_id
        transform.transform.translation.x = self._odom_msg.pose.pose.position.x
        transform.transform.translation.y = self._odom_msg.pose.pose.position.y
        transform.transform.translation.z = self._odom_msg.pose.pose.position.z
        transform.transform.rotation = self._odom_msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(transform)
        self._odom_pub.publish(self._odom_msg)
        self.get_logger().debug(
            'published odom x=%.3f y=%.3f yaw=%.3f vx=%.3f vy=%.3f wz=%.3f'
            % (
                self._odom_msg.pose.pose.position.x,
                self._odom_msg.pose.pose.position.y,
                msg.data[2] / 1000000.0,
                self._odom_msg.twist.twist.linear.x,
                self._odom_msg.twist.twist.linear.y,
                self._odom_msg.twist.twist.angular.z,
            )
        )

    def _response_quality(self, msg: Response) -> bool:
        expected_slave = self._expected_slave_id
        if msg.slave_id != expected_slave:
            return False
        if msg.func_code not in (0x03, 0x17):
            return False
        return len(msg.data) >= 8


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()