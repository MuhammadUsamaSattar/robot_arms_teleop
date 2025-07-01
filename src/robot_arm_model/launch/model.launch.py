from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import math


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_dir = FindPackageShare('robot_arm_model').find('robot_arm_model')

    #urdf_file = os.path.join(urdf_dir, 'urdf', 'so101_new_calib_right.urdf')
#
    #with open(urdf_file, 'r') as file:
    #    right_robot_desc = file.read()

    urdf_file = os.path.join(urdf_dir, 'urdf', 'so101_new_calib.urdf')

    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]),
        Node(
            package='robot_arm_model',
            executable='state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{'pos': 0.}]),
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('robot_arm_model'), 'rviz', 'default.rviz'])],
        ),
    ])