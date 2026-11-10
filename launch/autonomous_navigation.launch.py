from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigator')

    waypoints_path = LaunchConfiguration('waypoints_path')
    mission_loop = LaunchConfiguration('mission_loop')
    mission_pause_seconds = LaunchConfiguration('mission_pause_seconds')
    mission_frame_id = LaunchConfiguration('mission_frame_id')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_bringup.launch.py')
        )
    )

    waypoint_mission = Node(
        package='amr_navigator',
        executable='waypoint_mission.py',
        name='waypoint_mission',
        output='screen',
        parameters=[
            {
                'waypoints_path': waypoints_path,
                'loop': mission_loop,
                'pause_seconds': mission_pause_seconds,
                'frame_id': mission_frame_id,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'waypoints_path',
                default_value=os.path.join(pkg_share, 'path', 'waypoints.txt'),
                description='CSV waypoint file with x,y,z,qx,qy,qz,qw columns',
            ),
            DeclareLaunchArgument(
                'mission_loop',
                default_value='false',
                description='Repeat the waypoint mission forever',
            ),
            DeclareLaunchArgument(
                'mission_pause_seconds',
                default_value='0.5',
                description='Pause between waypoint goals',
            ),
            DeclareLaunchArgument(
                'mission_frame_id',
                default_value='map',
                description='Frame used for waypoint poses',
            ),
            nav2_bringup,
            waypoint_mission,
        ]
    )