from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigator')
    livox_share = get_package_share_directory('livox_mid360_mapping')
    om_share = get_package_share_directory('om_modbus_master')

    show_rviz = LaunchConfiguration('show_rviz')
    start_om_modbus_master = LaunchConfiguration('start_om_modbus_master')
    start_manual_drive = LaunchConfiguration('start_manual_drive')
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

    livox_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(livox_share, 'launch', 'bringup.launch.py')),
    )

    om_modbus_master = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(om_share, 'launch', 'om_modbus_master_launch.py')),
        condition=IfCondition(start_om_modbus_master),
        launch_arguments={
            'com': com,
            'topicID': topic_id,
            'baudrate': baudrate,
            'updateRate': update_rate,
            'firstGen': first_gen,
            'secondGen': second_gen,
            'globalID': global_id,
            'axisNum': axis_num,
        }.items(),
    )

    odom_tf = Node(
        package='amr_navigator',
        executable='om_odom_tf.py',
        output='screen',
        parameters=[
            {
                'topic_id': topic_id,
                'expected_slave_id': global_id,
                'accept_any_response': accept_any_response,
            }
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(start_om_modbus_master),
    )

    manual_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'joy_manual_drive.launch.py')),
        condition=IfCondition(start_manual_drive),
        launch_arguments={
            'topic_id': '1',
            'global_id': '10',
            'forward_axis': forward_axis,
            'turn_axis': turn_axis,
            'deadman_button': deadman_button,
            'speed_scale': speed_scale,
        }.items(),
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'livox.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription([
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
            'start_manual_drive',
            default_value='true',
            description='Start the manual joy ID-Share drive',
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
        livox_bringup,
        om_modbus_master,
        odom_tf,
        manual_drive,
        rviz_node,
    ])
