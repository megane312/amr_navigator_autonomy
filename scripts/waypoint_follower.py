#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional

import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry, Path as PathMsg
from om_msgs.msg import Query, State
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointFollower(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_follower')

        default_waypoints = str(
            Path(get_package_share_directory('amr_navigator')) / 'path' / 'recorded_waypoints.txt'
        )

        self.declare_parameter('waypoints_path', default_waypoints)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('topic_id', 1)
        self.declare_parameter('global_id', 10)
        self.declare_parameter('waypoint_tolerance', 0.30)
        self.declare_parameter('linear_gain', 0.60)
        self.declare_parameter('angular_gain', 1.8)
        self.declare_parameter('max_linear_mps', 0.20)
        self.declare_parameter('max_angular_rps', 1.00)
        self.declare_parameter('turn_in_place_threshold_rad', 0.80)
        self.declare_parameter('publish_period_sec', 0.05)
        self.declare_parameter('path_topic', '/waypoints')
        self.declare_parameter('status_topic', '/waypoint_follower/status')
        self.declare_parameter('target_marker_topic', '/waypoint_target_marker')
        self.declare_parameter('safety_stop_topic', '/waypoint_safety/stop')
        self.declare_parameter('wheel_radius', 0.143)
        self.declare_parameter('wheel_separation', 0.39)
        self.declare_parameter('gear_ratio', 30.0)
        self.declare_parameter('axis_num', 2)

        self._waypoints_path = Path(
            self.get_parameter('waypoints_path').get_parameter_value().string_value
        )
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._topic_id = int(self.get_parameter('topic_id').value)
        self._global_id = int(self.get_parameter('global_id').value)
        self._waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self._linear_gain = float(self.get_parameter('linear_gain').value)
        self._angular_gain = float(self.get_parameter('angular_gain').value)
        self._max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self._max_angular_rps = float(self.get_parameter('max_angular_rps').value)
        self._turn_in_place_threshold_rad = float(
            self.get_parameter('turn_in_place_threshold_rad').value
        )
        self._publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self._path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self._status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self._target_marker_topic = self.get_parameter('target_marker_topic').get_parameter_value().string_value
        self._safety_stop_topic = self.get_parameter('safety_stop_topic').get_parameter_value().string_value
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._wheel_separation = float(self.get_parameter('wheel_separation').value)
        self._gear_ratio = float(self.get_parameter('gear_ratio').value)
        self._axis_num = int(self.get_parameter('axis_num').value)

        self._set_param_client = self.create_client(SetParameters, '/om_node/set_parameters')
        self._initialized = False

        self._path_pub = self.create_publisher(
            PathMsg,
            self._path_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._status_pub = self.create_publisher(Float32MultiArray, self._status_topic, 10)
        self._target_marker_pub = self.create_publisher(Marker, self._target_marker_topic, 10)
        self._safety_stop_sub = self.create_subscription(Bool, self._safety_stop_topic, self._safety_stop_callback, 10)
        self._query_pub = self.create_publisher(Query, f'om_query{self._topic_id}', 10)
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10)
        self._state_sub = self.create_subscription(
            State, f'om_state{self._topic_id}', self._state_callback, 10
        )
        self._timer = self.create_timer(self._publish_period_sec, self._timer_callback)

        self._state_driver = 0
        self._odom: Optional[Odometry] = None
        self._waypoints = self._load_waypoints(self._waypoints_path)
        self._current_index = 0
        self._desired_linear = 0.0
        self._desired_angular = 0.0
        self._goal_reached = False
        self._safety_stop = False
        self._safety_stop_logged = False
        self._last_command: Optional[tuple[int, int]] = None
        self._current_distance = 0.0
        self._current_yaw_error = 0.0
        self._last_target_index = -1
        self._last_status_signature: Optional[tuple[int, int, int]] = None

        self._publish_path()
        self._publish_target_marker()
        self.get_logger().info(
            f'Loaded {len(self._waypoints)} waypoints from {self._waypoints_path.as_posix()}'
        )

    def _load_waypoints(self, waypoints_path: Path) -> List[tuple[float, float, float, float, float, float, float]]:
        if not waypoints_path.exists():
            raise FileNotFoundError(f'Waypoint file not found: {waypoints_path}')

        waypoints: List[tuple[float, float, float, float, float, float, float]] = []
        with waypoints_path.open('r', encoding='utf-8') as waypoint_file:
            for line_number, raw_line in enumerate(waypoint_file, start=1):
                line = raw_line.strip()
                if not line or line.startswith('#'):
                    continue

                values = [float(value) for value in line.split(',') if value.strip()]
                if len(values) != 7:
                    raise ValueError(
                        f'Unsupported waypoint format at line {line_number}: {raw_line.rstrip()}'
                    )
                waypoints.append(
                    (values[0], values[1], values[2], values[3], values[4], values[5], values[6])
                )

        if not waypoints:
            raise ValueError(f'No waypoints loaded from {waypoints_path}')

        return waypoints

    def _publish_path(self) -> None:
        path = PathMsg()
        path.header.frame_id = self._frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, z, qx, qy, qz, qw in self._waypoints:
            pose = self._pose_stamped(x, y, z, qx, qy, qz, qw)
            path.poses.append(pose)
        self._path_pub.publish(path)

    def _pose_stamped(self, x, y, z, qx, qy, qz, qw):
        from geometry_msgs.msg import PoseStamped, Quaternion as QuaternionMsg

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = QuaternionMsg(x=qx, y=qy, z=qz, w=qw)
        return pose

    def _state_callback(self, msg: State) -> None:
        self._state_driver = msg.state_driver

    def _safety_stop_callback(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)
        if self._safety_stop != self._safety_stop_logged:
            if self._safety_stop:
                self.get_logger().warning('Safety stop active, pausing waypoint motion')
            else:
                self.get_logger().info('Safety stop cleared, resuming waypoint motion')
            self._safety_stop_logged = self._safety_stop

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg
        self._update_command()

    def _update_command(self) -> None:
        if self._goal_reached or self._odom is None:
            self._desired_linear = 0.0
            self._desired_angular = 0.0
            return

        if self._safety_stop:
            self._hold_current_waypoint_state()
            return

        if self._current_index >= len(self._waypoints):
            self._goal_reached = True
            self._desired_linear = 0.0
            self._desired_angular = 0.0
            self._publish_status()
            self._publish_target_marker(goal_reached=True)
            return

        current = self._odom.pose.pose.position
        target = self._waypoints[self._current_index]
        target_x, target_y = target[0], target[1]

        distance = math.hypot(target_x - current.x, target_y - current.y)
        if distance <= self._waypoint_tolerance:
            self._current_distance = distance
            self._current_yaw_error = 0.0
            self._current_index += 1
            self._log_waypoint_progress(distance, reached=True)
            if self._current_index >= len(self._waypoints):
                self._goal_reached = True
                self._desired_linear = 0.0
                self._desired_angular = 0.0
                self._publish_status()
                self._publish_target_marker(goal_reached=True)
                return
            target = self._waypoints[self._current_index]
            target_x, target_y = target[0], target[1]
            distance = math.hypot(target_x - current.x, target_y - current.y)
            self._publish_target_marker()

        yaw = quaternion_to_yaw(self._odom.pose.pose.orientation)
        desired_heading = math.atan2(target_y - current.y, target_x - current.x)
        yaw_error = normalize_angle(desired_heading - yaw)
        self._current_distance = distance
        self._current_yaw_error = yaw_error

        angular = max(-self._max_angular_rps, min(self._max_angular_rps, self._angular_gain * yaw_error))
        linear = min(self._max_linear_mps, self._linear_gain * distance)
        if abs(yaw_error) > self._turn_in_place_threshold_rad:
            linear = 0.0
        else:
            linear *= max(0.0, math.cos(yaw_error))

        self._desired_linear = max(0.0, linear)
        self._desired_angular = angular
        self._publish_status()
        self._log_waypoint_progress(distance, reached=False)

    def _hold_current_waypoint_state(self) -> None:
        if self._odom is None or self._current_index >= len(self._waypoints):
            self._desired_linear = 0.0
            self._desired_angular = 0.0
            return

        current = self._odom.pose.pose.position
        target = self._waypoints[self._current_index]
        target_x, target_y = target[0], target[1]
        self._current_distance = math.hypot(target_x - current.x, target_y - current.y)

        yaw = quaternion_to_yaw(self._odom.pose.pose.orientation)
        desired_heading = math.atan2(target_y - current.y, target_x - current.x)
        self._current_yaw_error = normalize_angle(desired_heading - yaw)
        self._desired_linear = 0.0
        self._desired_angular = 0.0
        self._publish_status()

    def _publish_status(self) -> None:
        status = Float32MultiArray()
        status.data = [
            float(self._current_index + 1),
            float(len(self._waypoints)),
            float(self._current_distance),
            float(self._current_yaw_error),
            float(self._desired_linear),
            float(self._desired_angular),
            1.0 if self._goal_reached else 0.0,
        ]
        signature = (
            self._current_index,
            int(self._current_distance * 1000.0),
            int(self._current_yaw_error * 1000000.0),
        )
        if signature != self._last_status_signature:
            self._status_pub.publish(status)
            self._last_status_signature = signature

    def _publish_target_marker(self, goal_reached: bool = False) -> None:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoint_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETE if goal_reached else Marker.ADD
        if not goal_reached and self._current_index < len(self._waypoints):
            target = self._waypoints[self._current_index]
            marker.pose.position.x = target[0]
            marker.pose.position.y = target[1]
            marker.pose.position.z = target[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
        self._target_marker_pub.publish(marker)

    def _log_waypoint_progress(self, distance: float, reached: bool) -> None:
        target_index = min(self._current_index + 1, len(self._waypoints))
        if target_index != self._last_target_index or reached:
            self.get_logger().info(
                f'waypoint {target_index}/{len(self._waypoints)} '
                f'distance={distance:.3f}m yaw_error={self._current_yaw_error:.3f}rad '
                f'cmd=({self._desired_linear:.3f}m/s, {self._desired_angular:.3f}rad/s)'
            )
            self._last_target_index = target_index

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

    def _publish_query_delay(self, query: Query) -> None:
        self._query_pub.publish(query)
        time.sleep(0.02)

    def _write_single(self, slave_id: int, write_addr: int, value: int) -> None:
        query = Query()
        query.slave_id = slave_id
        query.func_code = 1
        query.write_addr = write_addr
        query.write_num = 1
        query.data[0] = value
        self._publish_query_delay(query)

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
            self._publish_query_delay(query)

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
            self._publish_query_delay(query)

    def _enable_drives(self) -> None:
        self._set_remote_global_id(-1)
        self._write_single(1, 124, 1)
        self._write_single(2, 124, 1)
        self._set_share_data()
        self._set_remote_global_id(self._global_id)
        self.get_logger().info('ID Share setup completed')

    def _publish_drive(self, linear_mps: float, angular_rps: float) -> None:
        # Calculate wheel velocities (m/s)
        v_r = linear_mps + (self._wheel_separation / 2.0) * angular_rps
        v_l = linear_mps - (self._wheel_separation / 2.0) * angular_rps

        # Convert to motor RPM (r/min)
        # RPM = v_wheel * 60 / (2 * pi * radius) * gear_ratio
        coef = (60.0 * self._gear_ratio) / (2.0 * math.pi * self._wheel_radius)
        right_rpm = int(v_r * coef)
        left_rpm = int(v_l * coef)

        # Compose write query targeting global ID 10, write_addr 0x0000, write_num 12
        query = Query()
        query.slave_id = self._global_id
        query.func_code = 1
        query.write_addr = 0x0000
        query.write_num = 12

        # Right motor (ID 1)
        query.data[0] = 16
        query.data[1] = 1000
        query.data[2] = right_rpm
        query.data[3] = 50000
        query.data[4] = 80000
        query.data[5] = 1

        # Left motor (ID 2)
        query.data[6] = 16
        query.data[7] = 1000
        query.data[8] = left_rpm
        query.data[9] = 50000
        query.data[10] = 80000
        query.data[11] = 1

        self._query_pub.publish(query)

        drive_key = (right_rpm, left_rpm)
        if drive_key != self._last_command:
            self.get_logger().info(
                f'waypoint idx={self._current_index + 1}/{len(self._waypoints)} '
                f'linear={linear_mps:.3f} m/s angular={angular_rps:.3f} rad/s '
                f'left={left_rpm} rpm right={right_rpm} rpm'
            )
            self._last_command = drive_key

    def _timer_callback(self) -> None:
        if not self._initialized:
            return

        if self._state_driver == 1:
            return

        if self._safety_stop:
            self._publish_drive(0.0, 0.0)
            return

        if self._goal_reached:
            if self._last_command != (0, 0):
                self.get_logger().info('Final waypoint reached, stopping the robot')
                self._last_command = (0, 0)
            self._publish_drive(0.0, 0.0)
            return

        self._publish_drive(self._desired_linear, self._desired_angular)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointFollower()
        node.bootstrap()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()