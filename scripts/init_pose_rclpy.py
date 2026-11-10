#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class InitPosePublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.pub = self.create_publisher(Float32MultiArray, '/motor_command', 10)
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = ''
        dim.size = 0
        dim.stride = 0
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = [0, 0]
        self.msg = msg
        self.count = 0
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        if self.count < 5:
            self.pub.publish(self.msg)
            self.count += 1
        else:
            self.get_logger().info('Published 5 messages, shutting down')
            self.timer.cancel()
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
