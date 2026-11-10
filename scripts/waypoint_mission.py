#!/usr/bin/env python3

from pathlib import Path
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class WaypointMission(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_mission')

        default_waypoints = str(
            Path(get_package_share_directory('amr_navigator')) / 'path' / 'recorded_waypoints.txt'
        )

        self.declare_parameter('waypoints_path', default_waypoints)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('loop', False)
        self.declare_parameter('pause_seconds', 0.5)
        self.declare_parameter('path_topic', '/waypoints')
        self.declare_parameter('publish_only', False)

        self._waypoints_path = Path(
            self.get_parameter('waypoints_path').get_parameter_value().string_value
        )
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._loop = self.get_parameter('loop').get_parameter_value().bool_value
        self._pause_seconds = (
            self.get_parameter('pause_seconds').get_parameter_value().double_value
        )
        self._path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self._publish_only = self.get_parameter('publish_only').get_parameter_value().bool_value

        self._path_publisher = self.create_publisher(
            PathMsg,
            self._path_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._action_client = None if self._publish_only else ActionClient(self, NavigateToPose, self._action_name)

        if not self._waypoints_path.exists():
            fallback_path = self._waypoints_path.with_name('waypoints.txt')
            if fallback_path.exists():
                self.get_logger().warning(
                    f'Waypoint file not found: {self._waypoints_path}. Falling back to {fallback_path}'
                )
                self._waypoints_path = fallback_path

        self._waypoints = self._load_waypoints(self._waypoints_path)
        self._publish_waypoints()

    def _load_waypoints(self, waypoints_path: Path) -> List[PoseStamped]:
        if not waypoints_path.exists():
            raise FileNotFoundError(f'Waypoint file not found: {waypoints_path}')

        waypoints: List[PoseStamped] = []
        with waypoints_path.open('r', encoding='utf-8') as waypoint_file:
            for line_number, raw_line in enumerate(waypoint_file, start=1):
                line = raw_line.strip()
                if not line or line.startswith('#'):
                    continue

                values = [float(value) for value in line.split(',') if value.strip()]

                pose = PoseStamped()
                pose.header.frame_id = self._frame_id

                if len(values) == 7:
                    pose.pose.position.x = values[0]
                    pose.pose.position.y = values[1]
                    pose.pose.position.z = values[2]
                    pose.pose.orientation = Quaternion(
                        x=values[3], y=values[4], z=values[5], w=values[6]
                    )
                elif len(values) == 3:
                    pose.pose.position.x = values[0]
                    pose.pose.position.y = values[1]
                    pose.pose.position.z = values[2]
                    pose.pose.orientation.w = 1.0
                elif len(values) == 2:
                    pose.pose.position.x = values[0]
                    pose.pose.position.y = values[1]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                else:
                    raise ValueError(
                        f'Unsupported waypoint format at line {line_number}: {raw_line.rstrip()}'
                    )

                waypoints.append(pose)

        if not waypoints:
            raise ValueError(f'No waypoints loaded from {waypoints_path}')

        self.get_logger().info(
            f'Loaded {len(waypoints)} waypoints from {waypoints_path.as_posix()}'
        )
        return waypoints

    def _publish_waypoints(self) -> None:
        path = PathMsg()
        path.header.frame_id = self._frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = list(self._waypoints)
        self._path_publisher.publish(path)

    def _wait_for_action_server(self) -> None:
        if self._publish_only:
            return

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {self._action_name} action server...')

    def _send_goal(self, pose: PoseStamped) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose.pose

        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal was rejected')
            return False

        self.get_logger().info('NavigateToPose goal accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error('No result returned from NavigateToPose')
            return False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint reached successfully')
            return True

        self.get_logger().error(f'Waypoint failed with status code {result.status}')
        return False

    def run(self) -> None:
        if self._publish_only:
            self.get_logger().info('Publish-only mode: keeping waypoint Path latched for RViz')
            rclpy.spin(self)
            return

        self._wait_for_action_server()

        while rclpy.ok():
            for index, pose in enumerate(self._waypoints, start=1):
                self.get_logger().info(
                    f'Navigating to waypoint {index}/{len(self._waypoints)}'
                )
                if not self._send_goal(pose):
                    return

                if self._pause_seconds > 0.0:
                    self.get_logger().info(
                        f'Waiting {self._pause_seconds:.1f} seconds before next waypoint'
                    )
                    rclpy.spin_once(self, timeout_sec=self._pause_seconds)

            if not self._loop:
                return

            self.get_logger().info('Restarting waypoint mission from the beginning')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = WaypointMission()
        node.run()
    except (FileNotFoundError, ValueError) as exc:
        rclpy.logging.get_logger('waypoint_mission').error(str(exc))
        raise SystemExit(1) from exc
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()