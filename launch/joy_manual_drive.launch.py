from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_id = LaunchConfiguration('topic_id')
    global_id = LaunchConfiguration('global_id')
    axis_num = LaunchConfiguration('axis_num')
    joy_topic = LaunchConfiguration('joy_topic')
    forward_axis = LaunchConfiguration('forward_axis')
    turn_axis = LaunchConfiguration('turn_axis')
    deadman_button = LaunchConfiguration('deadman_button')
    max_forward_rpm = LaunchConfiguration('max_forward_rpm')
    max_turn_rpm = LaunchConfiguration('max_turn_rpm')
    speed_scale = LaunchConfiguration('speed_scale')

    return LaunchDescription([
        DeclareLaunchArgument('topic_id', default_value='1'),
        DeclareLaunchArgument('global_id', default_value='10'),
        DeclareLaunchArgument('axis_num', default_value='2'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('forward_axis', default_value='1'),
        DeclareLaunchArgument('turn_axis', default_value='0'),
        DeclareLaunchArgument('deadman_button', default_value='4'),
        DeclareLaunchArgument('max_forward_rpm', default_value='360.0'),
        DeclareLaunchArgument('max_turn_rpm', default_value='180.0'),
        DeclareLaunchArgument('speed_scale', default_value='2.0'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='amr_navigator',
            executable='joy_idshare_drive.py',
            name='joy_idshare_drive',
            output='screen',
            parameters=[{
                'topic_id': topic_id,
                'global_id': global_id,
                'axis_num': axis_num,
                'joy_topic': joy_topic,
                'forward_axis': forward_axis,
                'turn_axis': turn_axis,
                'deadman_button': deadman_button,
                'max_forward_rpm': max_forward_rpm,
                'max_turn_rpm': max_turn_rpm,
                'speed_scale': speed_scale,
            }],
        ),
    ])