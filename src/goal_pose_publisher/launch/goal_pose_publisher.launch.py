import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_file = LaunchConfiguration('video_file')
    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_file',
            default_value = 'Video.mp4',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value = 'LIVE_STREAM',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', '--roll', str(0), '--pitch', str(0), '--yaw', str(0),
                '--frame-id', 'right_panda_hand', '--child-frame-id', 'right_goal_pose_frame'
            ],
            parameters=[
                {'use_sim_time': False}
                        ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', '--roll', str(0), '--pitch', str(0), '--yaw', str(0),
                '--frame-id', 'left_panda_hand', '--child-frame-id', 'left_goal_pose_frame'
            ],
            parameters=[
                {'use_sim_time': False}
                        ],
        ),
        Node(
            package='goal_pose_publisher',
            executable='landmark_publisher',
            parameters=[
                {'video_file': video_file,
                 'mode': mode,
                 'use_sim_time': False}
                 
                ]
        ),
        Node(
            package='goal_pose_publisher',
            executable='goal_pose_publisher',
            namespace='right',
            parameters=[
                {'use_sim_time': False}
                        ],
        ),
        Node(
            package='goal_pose_publisher',
            executable='goal_pose_publisher',
            namespace='left',
            parameters=[
                {'use_sim_time': False}
                        ],
        ),
    ])