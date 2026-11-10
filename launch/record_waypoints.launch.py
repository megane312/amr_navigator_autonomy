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
    topic_id = LaunchConfiguration('topic_id')
    global_id = LaunchConfiguration('global_id')
    axis_num = LaunchConfiguration('axis_num')
    com = LaunchConfiguration('com')
    baudrate = LaunchConfiguration('baudrate')
    update_rate = LaunchConfiguration('update_rate')
    first_gen = LaunchConfiguration('first_gen')
    second_gen = LaunchConfiguration('second_gen')
    forward_axis = LaunchConfiguration('forward_axis')
    turn_axis = LaunchConfiguration('turn_axis')
    deadman_button = LaunchConfiguration('deadman_button')
    speed_scale = LaunchConfiguration('speed_scale')
    accept_any_response = LaunchConfiguration('accept_any_response')
    waypoint_file = LaunchConfiguration('waypoint_file')
    pose_topic = LaunchConfiguration('pose_topic')
    marker_frame_id = LaunchConfiguration('marker_frame_id')
    waypoint_interval = LaunchConfiguration('waypoint_interval')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'map_livox_fastlio_gmap.launch.py')
        ),
        launch_arguments={
            'show_rviz': show_rviz,
            'start_om_modbus_master': start_om_modbus_master,
            'com': com,
            'topic_id': topic_id,
            'global_id': global_id,
            'axis_num': axis_num,
            'baudrate': baudrate,
            'update_rate': update_rate,
            'first_gen': first_gen,
            'second_gen': second_gen,
            'forward_axis': forward_axis,
            'turn_axis': turn_axis,
            'deadman_button': deadman_button,
            'speed_scale': speed_scale,
            'accept_any_response': accept_any_response,
        }.items(),
    )

    odom_to_pose = Node(
        package='amr_navigator',
        executable='odom_to_pose',
        name='odom_to_pose',
        output='screen',
        remappings=[
            ('/odom', '/Odometry'),
        ]
    )

    waypoint_recorder = Node(
        package='amr_navigator',
        executable='waypoint_recorder',
        name='waypoint_recorder',
        output='screen',
        parameters=[
            {
                'pose_topic': pose_topic,
                'waypoint_file': waypoint_file,
                'waypoint_interval': waypoint_interval,
                'frame_id': marker_frame_id,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'show_rviz',
                default_value='true',
                description='Start RViz with the Livox FAST_LIO config',
            ),
            DeclareLaunchArgument(
                'start_om_modbus_master',
                default_value='true',
                description='Start the OM Modbus driver for motor control',
            ),
            DeclareLaunchArgument(
                'com',
                default_value='/dev/ttyUSB0',
                description='Serial port for OM Modbus master',
            ),
            DeclareLaunchArgument(
                'topic_id',
                default_value='1',
                description='OM Modbus topic ID',
            ),
            DeclareLaunchArgument(
                'global_id',
                default_value='10',
                description='ID-Share global ID',
            ),
            DeclareLaunchArgument(
                'axis_num',
                default_value='2',
                description='Number of axes to configure in ID-Share mode',
            ),
            DeclareLaunchArgument(
                'baudrate',
                default_value='230400',
                description='Serial baudrate for OM Modbus master',
            ),
            DeclareLaunchArgument(
                'update_rate',
                default_value='1000',
                description='OM Modbus update rate',
            ),
            DeclareLaunchArgument(
                'first_gen',
                default_value='',
                description='First generation axes list',
            ),
            DeclareLaunchArgument(
                'second_gen',
                default_value='1,2,',
                description='Second generation axes list',
            ),
            DeclareLaunchArgument(
                'accept_any_response',
                default_value='false',
                description='Allow om_odom_tf to accept any response for debugging',
            ),
            DeclareLaunchArgument(
                'forward_axis',
                default_value='1',
                description='Joystick axis for forward/backward motion',
            ),
            DeclareLaunchArgument(
                'turn_axis',
                default_value='0',
                description='Joystick axis for left/right turn',
            ),
            DeclareLaunchArgument(
                'deadman_button',
                default_value='4',
                description='Button that must be held to drive',
            ),
            DeclareLaunchArgument(
                'speed_scale',
                default_value='2.0',
                description='Multiplier for final motor RPM output',
            ),
            DeclareLaunchArgument(
                'waypoint_file',
                default_value=os.path.join(pkg_share, 'path', 'recorded_waypoints.txt'),
                description='CSV file to append waypoint records to',
            ),
            DeclareLaunchArgument(
                'pose_topic',
                default_value='/pose',
                description='Pose topic used by waypoint_recorder',
            ),
            DeclareLaunchArgument(
                'marker_frame_id',
                default_value='map',
                description='Frame used to visualize recorded waypoints',
            ),
            DeclareLaunchArgument(
                'waypoint_interval',
                default_value='0.25',
                description='Minimum spacing between recorded waypoints',
            ),
            mapping_launch,
            odom_to_pose,
            waypoint_recorder,
        ]
    )