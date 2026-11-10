from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigator')

    show_rviz = LaunchConfiguration('show_rviz')
    start_om_modbus_master = LaunchConfiguration('start_om_modbus_master')
    start_manual_drive = LaunchConfiguration('start_manual_drive')
    waypoint_file = LaunchConfiguration('waypoint_file')
    waypoint_tolerance = LaunchConfiguration('waypoint_tolerance')
    linear_gain = LaunchConfiguration('linear_gain')
    angular_gain = LaunchConfiguration('angular_gain')
    max_linear_mps = LaunchConfiguration('max_linear_mps')
    max_angular_rps = LaunchConfiguration('max_angular_rps')
    turn_in_place_threshold_rad = LaunchConfiguration('turn_in_place_threshold_rad')
    status_topic = LaunchConfiguration('status_topic')
    target_marker_topic = LaunchConfiguration('target_marker_topic')
    safety_stop_topic = LaunchConfiguration('safety_stop_topic')
    obstacle_count_topic = LaunchConfiguration('obstacle_count_topic')
    obstacle_marker_topic = LaunchConfiguration('obstacle_marker_topic')
    obstacle_cloud_topic = LaunchConfiguration('obstacle_cloud_topic')
    obstacle_frame_id = LaunchConfiguration('obstacle_frame_id')
    obstacle_forward_range = LaunchConfiguration('obstacle_forward_range')
    obstacle_lateral_range = LaunchConfiguration('obstacle_lateral_range')
    obstacle_z_min = LaunchConfiguration('obstacle_z_min')
    obstacle_z_max = LaunchConfiguration('obstacle_z_max')
    obstacle_stop_count = LaunchConfiguration('obstacle_stop_count')
    obstacle_resume_count = LaunchConfiguration('obstacle_resume_count')
    obstacle_voxel_leaf = LaunchConfiguration('obstacle_voxel_leaf')
    topic_id = LaunchConfiguration('topic_id')
    global_id = LaunchConfiguration('global_id')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'map_livox_fastlio_gmap.launch.py')),
        launch_arguments={
            'show_rviz': show_rviz,
            'start_om_modbus_master': start_om_modbus_master,
            'start_manual_drive': start_manual_drive,
            'topic_id': topic_id,
            'global_id': global_id,
            'axis_num': '2',
            'com': '/dev/ttyUSB0',
            'baudrate': '230400',
            'update_rate': '1000',
            'first_gen': '',
            'second_gen': '1,2,',
            'forward_axis': '1',
            'turn_axis': '0',
            'deadman_button': '4',
            'speed_scale': '2.0',
            'accept_any_response': 'false',
        }.items(),
    )

    waypoint_follower = Node(
        package='amr_navigator',
        executable='waypoint_follower.py',
        name='waypoint_follower',
        output='screen',
        parameters=[
            {
                'waypoints_path': waypoint_file,
                'frame_id': 'map',
                'odom_topic': '/Odometry',
                'topic_id': topic_id,
                'global_id': global_id,
                'waypoint_tolerance': waypoint_tolerance,
                'linear_gain': linear_gain,
                'angular_gain': angular_gain,
                'max_linear_mps': max_linear_mps,
                'max_angular_rps': max_angular_rps,
                'turn_in_place_threshold_rad': turn_in_place_threshold_rad,
                'path_topic': '/waypoints',
                'status_topic': status_topic,
                'target_marker_topic': target_marker_topic,
                'safety_stop_topic': safety_stop_topic,
            }
        ],
    )

    obstacle_stop_monitor = Node(
        package='amr_navigator',
        executable='obstacle_stop_monitor.py',
        name='obstacle_stop_monitor',
        output='screen',
        parameters=[
            {
                'cloud_topic': obstacle_cloud_topic,
                'stop_topic': safety_stop_topic,
                'count_topic': obstacle_count_topic,
                'marker_topic': obstacle_marker_topic,
                'frame_id': obstacle_frame_id,
                'forward_range': obstacle_forward_range,
                'lateral_range': obstacle_lateral_range,
                'z_min': obstacle_z_min,
                'z_max': obstacle_z_max,
                'stop_count': obstacle_stop_count,
                'resume_count': obstacle_resume_count,
                'voxel_leaf': obstacle_voxel_leaf,
                'ground_filter_slope': LaunchConfiguration('obstacle_ground_filter_slope'),
                'ground_filter_intercept': LaunchConfiguration('obstacle_ground_filter_intercept'),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('show_rviz', default_value='true'),
            DeclareLaunchArgument(
                'start_om_modbus_master',
                default_value='true',
                description='Start the OM Modbus driver for autonomous waypoint following',
            ),
            DeclareLaunchArgument(
                'start_manual_drive',
                default_value='false',
                description='Start the joystick manual drive (kept off for autonomous mode)',
            ),
            DeclareLaunchArgument(
                'waypoint_file',
                default_value=os.path.join(pkg_share, 'path', 'recorded_waypoints.txt'),
                description='CSV waypoint file to follow',
            ),
            DeclareLaunchArgument('waypoint_tolerance', default_value='0.30'),
            DeclareLaunchArgument('linear_gain', default_value='0.60'),
            DeclareLaunchArgument('angular_gain', default_value='1.80'),
            DeclareLaunchArgument('max_linear_mps', default_value='0.20'),
            DeclareLaunchArgument('max_angular_rps', default_value='1.00'),
            DeclareLaunchArgument('turn_in_place_threshold_rad', default_value='0.80'),
            DeclareLaunchArgument('status_topic', default_value='/waypoint_follower/status'),
            DeclareLaunchArgument('target_marker_topic', default_value='/waypoint_target_marker'),
            DeclareLaunchArgument('safety_stop_topic', default_value='/waypoint_safety/stop'),
            DeclareLaunchArgument('obstacle_count_topic', default_value='/waypoint_safety/obstacle_count'),
            DeclareLaunchArgument('obstacle_marker_topic', default_value='/waypoint_safety_marker'),
            DeclareLaunchArgument('obstacle_cloud_topic', default_value='/cloud_registered_body'),
            DeclareLaunchArgument('obstacle_frame_id', default_value='base_link'),
            DeclareLaunchArgument('obstacle_forward_range', default_value='0.8'),
            DeclareLaunchArgument('obstacle_lateral_range', default_value='0.35'),
            DeclareLaunchArgument('obstacle_z_min', default_value='0.1'),
            DeclareLaunchArgument('obstacle_z_max', default_value='1.8'),
            DeclareLaunchArgument('obstacle_stop_count', default_value='6'),
            DeclareLaunchArgument('obstacle_resume_count', default_value='2'),
            DeclareLaunchArgument('obstacle_voxel_leaf', default_value='0.10'),
            DeclareLaunchArgument('obstacle_ground_filter_slope', default_value='-0.27'),
            DeclareLaunchArgument('obstacle_ground_filter_intercept', default_value='0.52'),
            DeclareLaunchArgument('topic_id', default_value='1'),
            DeclareLaunchArgument('global_id', default_value='10'),
            base_bringup,
            obstacle_stop_monitor,
            waypoint_follower,
        ]
    )
