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
                '--x', '0.278', '--y', '-0.300', '--z', '0.666', 
                '--qx', '0.5', '--qy', '0.5',
                '--qz', '0.5', '--qw', '0.5',
                '--frame-id', 'combined_base', '--child-frame-id', 'right_goal_pose_frame'
            ],
            parameters=[
                {'use_sim_time': False}
                        ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.278', '--y', '0.300', '--z', '0.666', 
                '--qx', '0.5', '--qy', '0.5',
                '--qz', '0.5', '--qw', '0.5',
                '--frame-id', 'combined_base', '--child-frame-id', 'left_goal_pose_frame'
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
            parameters=[
                {'use_sim_time': False}
                        ],
        ),
    ])