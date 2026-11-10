from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigator')

    waypoints_path = LaunchConfiguration('waypoints_path')
    frame_id = LaunchConfiguration('frame_id')
    publish_only = LaunchConfiguration('publish_only')
    show_rviz = LaunchConfiguration('show_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    waypoint_mission = Node(
        package='amr_navigator',
        executable='waypoint_mission.py',
        name='waypoint_mission',
        output='screen',
        parameters=[
            {
                'waypoints_path': waypoints_path,
                'frame_id': frame_id,
                'publish_only': publish_only,
                'path_topic': '/waypoints',
            }
        ],
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'waypoints_path',
                default_value=os.path.join(pkg_share, 'path', 'recorded_waypoints.txt'),
                description='CSV waypoint file to replay',
            ),
            DeclareLaunchArgument(
                'frame_id',
                default_value='map',
                description='Frame used for waypoint poses',
            ),
            DeclareLaunchArgument(
                'publish_only',
                default_value='true',
                description='Keep publishing the Path for RViz without sending Nav2 goals',
            ),
            DeclareLaunchArgument(
                'show_rviz',
                default_value='true',
                description='Start RViz with the path display',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=os.path.join(pkg_share, 'rviz', 'path.rviz'),
                description='RViz configuration file',
            ),
            waypoint_mission,
            rviz_node,
        ]
    )