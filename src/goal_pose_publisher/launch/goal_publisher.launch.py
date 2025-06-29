from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_pose_publisher',
            executable='landmark_publisher',
        ),
        Node(
            package='goal_pose_publisher',
            executable='goal_pose_publisher',
        ),
    ])