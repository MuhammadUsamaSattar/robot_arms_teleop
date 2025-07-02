import math

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', '--yaw', str(math.pi/2),
                '--frame-id', 'right_gripper', '--child-frame-id', 'right_goal_pose_frame'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0', '--yaw', str(math.pi/2),
                '--frame-id', 'left_gripper', '--child-frame-id', 'left_goal_pose_frame'
            ]
        ),
        Node(
            package='goal_pose_publisher',
            executable='landmark_publisher',
        ),
        Node(
            package='goal_pose_publisher',
            executable='goal_pose_publisher',
            namespace='right',
        ),
        Node(
            package='goal_pose_publisher',
            executable='goal_pose_publisher',
            namespace='left',
        ),
    ])