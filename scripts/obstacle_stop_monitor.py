#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker


class ObstacleStopMonitor(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_stop_monitor')

        self.declare_parameter('cloud_topic', '/cloud_registered_body')
        self.declare_parameter('stop_topic', '/waypoint_safety/stop')
        self.declare_parameter('count_topic', '/waypoint_safety/obstacle_count')
        self.declare_parameter('marker_topic', '/waypoint_safety_marker')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('forward_range', 1.4)
        self.declare_parameter('lateral_range', 0.55)
        self.declare_parameter('z_min', 0.1)
        self.declare_parameter('z_max', 1.8)
        self.declare_parameter('stop_count', 6)
        self.declare_parameter('resume_count', 2)
        self.declare_parameter('voxel_leaf', 0.10)
        self.declare_parameter('ground_filter_slope', 0.0)
        self.declare_parameter('ground_filter_intercept', 0.0)

        self._cloud_topic = self.get_parameter('cloud_topic').value
        self._stop_topic = self.get_parameter('stop_topic').value
        self._count_topic = self.get_parameter('count_topic').value
        self._marker_topic = self.get_parameter('marker_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._forward_range = float(self.get_parameter('forward_range').value)
        self._lateral_range = float(self.get_parameter('lateral_range').value)
        self._z_min = float(self.get_parameter('z_min').value)
        self._z_max = float(self.get_parameter('z_max').value)
        self._stop_count = int(self.get_parameter('stop_count').value)
        self._resume_count = int(self.get_parameter('resume_count').value)
        self._voxel_leaf = float(self.get_parameter('voxel_leaf').value)
        self._ground_filter_slope = float(self.get_parameter('ground_filter_slope').value)
        self._ground_filter_intercept = float(self.get_parameter('ground_filter_intercept').value)

        self._stop_pub = self.create_publisher(Bool, self._stop_topic, 10)
        self._count_pub = self.create_publisher(Float32, self._count_topic, 10)
        self._marker_pub = self.create_publisher(Marker, self._marker_topic, 10)
        self._cloud_sub = self.create_subscription(PointCloud2, self._cloud_topic, self._cloud_callback, 10)

        self._stop_active = False
        self._last_stop_sent: bool | None = None

        self.get_logger().info(
            f'Watching {self._cloud_topic} for obstacles: forward_range={self._forward_range:.2f} '
            f'lateral_range={self._lateral_range:.2f} stop_count={self._stop_count} resume_count={self._resume_count}'
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        obstacle_count = 0
        closest_distance = float('inf')
        debug_points = []

        sample_step = 1
        if self._voxel_leaf >= 0.20:
            sample_step = 3
        elif self._voxel_leaf >= 0.10:
            sample_step = 2

        for index, point in enumerate(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)):
            if index % sample_step != 0:
                continue
            x, y, z = point
            z_ground = self._ground_filter_slope * x + self._ground_filter_intercept
            if z < (z_ground + self._z_min) or z > self._z_max:
                continue
            if x < 0.0 or x > self._forward_range:
                continue
            if abs(y) > self._lateral_range:
                continue

            obstacle_count += 1
            closest_distance = min(closest_distance, math.hypot(x, y))
            if len(debug_points) < 5:
                debug_points.append((x, y, z))

        if self._stop_active:
            if obstacle_count <= self._resume_count:
                self._stop_active = False
        else:
            if obstacle_count >= self._stop_count:
                self._stop_active = True

        self._publish_state(obstacle_count, closest_distance, debug_points)

    def _publish_state(self, obstacle_count: int, closest_distance: float, debug_points: list) -> None:
        stop_msg = Bool()
        stop_msg.data = self._stop_active
        self._stop_pub.publish(stop_msg)

        count_msg = Float32()
        count_msg.data = float(obstacle_count)
        self._count_pub.publish(count_msg)

        if self._stop_active != self._last_stop_sent:
            if self._stop_active:
                self.get_logger().warning(
                    f'Obstacle detected: count={obstacle_count} closest={closest_distance:.2f}m -> STOP'
                )
                self.get_logger().warning(
                    f'Sample detected points (x, y, z): {[(round(p[0], 3), round(p[1], 3), round(p[2], 3)) for p in debug_points]}'
                )
            else:
                self.get_logger().info(
                    f'Obstacle cleared: count={obstacle_count} closest={closest_distance:.2f}m -> RESUME'
                )
            self._last_stop_sent = self._stop_active

        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoint_safety'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self._forward_range / 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = (self._z_min + self._z_max) / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._forward_range
        marker.scale.y = self._lateral_range * 2.0
        marker.scale.z = self._z_max - self._z_min
        marker.color.a = 0.18
        marker.color.r = 1.0 if self._stop_active else 0.0
        marker.color.g = 0.0 if self._stop_active else 1.0
        marker.color.b = 0.0
        self._marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleStopMonitor()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
