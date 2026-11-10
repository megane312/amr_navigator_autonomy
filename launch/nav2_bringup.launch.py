from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigator')
    nav2_share = get_package_share_directory('nav2_bringup')
    om_share = get_package_share_directory('om_modbus_master')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    start_om_modbus_master = LaunchConfiguration('start_om_modbus_master')
    start_manual_drive = LaunchConfiguration('start_manual_drive')
    bringup_preset = LaunchConfiguration('bringup_preset')
    speed_scale = LaunchConfiguration('speed_scale')
    start_nav_stack = LaunchConfiguration('start_nav_stack')
    publish_map_to_odom_static = LaunchConfiguration('publish_map_to_odom_static')
    publish_odom_to_base_static = LaunchConfiguration('publish_odom_to_base_static')
    accept_any_response = LaunchConfiguration('accept_any_response')
    accept_any_address = LaunchConfiguration('accept_any_address')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        condition=IfCondition(start_nav_stack),
        launch_arguments={
            'namespace': '',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
            'container_name': 'nav2_container',
        }.items(),
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'nav2_navigation.launch.py')),
        condition=IfCondition(start_nav_stack),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    om_modbus_master = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(om_share, 'launch', 'om_modbus_master_launch.py')),
        condition=IfCondition(start_om_modbus_master),
            launch_arguments={
                'com': '/dev/ttyUSB0',
                'topicID': '1',
                'baudrate': '230400',
                'updateRate': '1000',
                'firstGen': '',
                'secondGen': '1,2,',
                'globalID': '10',
                'axisNum': '2',
                'accept_any_address': accept_any_address,
            }.items(),
    )

    odom_tf = Node(
        package='amr_navigator',
        executable='om_odom_tf.py',
        output='screen',
        parameters=[{'accept_any_response': accept_any_response}],
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(start_om_modbus_master),
    )

    manual_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'joy_manual_drive.launch.py')),
        condition=IfCondition(start_manual_drive),
        launch_arguments={
            'topic_id': '1',
            'global_id': '10',
            'speed_scale': speed_scale,
        }.items(),
    )

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(start_nav_stack),
    )

    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        condition=IfCondition(start_nav_stack),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bringup_preset',
            default_value='nav_manual',
            description='Bringup preset: nav_only, manual_drive, or nav_manual',
        ),
        DeclareLaunchArgument(
            'start_nav_stack',
            default_value=PythonExpression(["'", bringup_preset, "' in ['nav_only', 'nav_manual']"]),
            description='Start Nav2 localization and navigation stack',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'data', 'robot_map.yaml'),
            description='Full path to map yaml file to load',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for Nav2',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='false',
            description='Use composed bringup if true',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Respawn crashed nodes when composition is disabled',
        ),
        DeclareLaunchArgument(
            'start_om_modbus_master',
            default_value=PythonExpression(["'", bringup_preset, "' in ['manual_drive', 'nav_manual']"]),
            description='Start the OM Modbus driver and odom source',
        ),
        DeclareLaunchArgument(
            'start_manual_drive',
            default_value=PythonExpression(["'", bringup_preset, "' in ['manual_drive', 'nav_manual']"]),
            description='Start the manual joy ID-Share drive',
        ),
        DeclareLaunchArgument(
            'publish_map_to_odom_static',
            default_value='true',
            description='Publish a temporary static transform from map to odom for testing',
        ),
        DeclareLaunchArgument(
            'publish_odom_to_base_static',
            default_value='false',
            description='Publish a temporary static transform from odom to base_link for testing',
        ),
        DeclareLaunchArgument(
            'accept_any_response',
            default_value='false',
            description='Allow om_odom_tf to publish odom from any response for debugging',
        ),
        DeclareLaunchArgument(
            'accept_any_address',
            default_value='false',
            description='Allow om_modbus_master to accept any address for debugging',
        ),
        DeclareLaunchArgument(
            'speed_scale',
            default_value='2.0',
            description='Global speed scale multiplier for manual drive',
        ),
        om_modbus_master,
        manual_drive,
        odom_tf,
        static_map_to_odom,
        static_odom_to_base,
        nav2_launch,
        nav2_navigation,
    ])
