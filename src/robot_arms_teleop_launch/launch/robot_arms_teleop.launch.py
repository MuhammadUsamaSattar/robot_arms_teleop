from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    video_file = LaunchConfiguration('video_file')
    mode = LaunchConfiguration('mode')

    launch_dir_motion_planning_moveit_config = PathJoinSubstitution([FindPackageShare('motion_planning_moveit_config'), 'launch'])
    launch_dir_motion_planning = PathJoinSubstitution([FindPackageShare('motion_planning'), 'launch'])
    launch_dir_goal_pose_publisher = PathJoinSubstitution([FindPackageShare('goal_pose_publisher'), 'launch'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_file',
            default_value = 'Video.mp4',
        ),
        DeclareLaunchArgument(
            'mode',
            default_value = 'LIVE_STREAM',
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir_motion_planning_moveit_config, 'demo.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir_motion_planning, 'motion_planning.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir_goal_pose_publisher, 'goal_pose_publisher.launch.py']),
            launch_arguments={'video_file': video_file, 'mode': mode}.items()
        ),
    ])