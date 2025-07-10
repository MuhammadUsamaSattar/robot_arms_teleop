from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("motion_planning").to_moveit_configs()

    motion_planning_node_left = Node(
        package="motion_planning",
        executable="motion_planning",
        namespace="left",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ],
        remappings=[
        ('joint_states', '/joint_states')
        ],
    )
    #motion_planning_node_right = Node(
    #    package="motion_planning",
    #    executable="motion_planning",
    #    namespace="right",
    #    output="screen",
    #    parameters=[
    #        moveit_config.robot_description,
    #        moveit_config.robot_description_semantic,
    #        moveit_config.robot_description_kinematics,
    #        {'use_sim_time': False},
    #    ],
    #    remappings=[
    #    ('joint_states', '/joint_states')
    #    ],
    #)

    return LaunchDescription([motion_planning_node_left])#, motion_planning_node_right])