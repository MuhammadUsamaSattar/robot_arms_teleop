import math

import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_arms_teleop_interfaces.msg import Landmarks
from std_msgs.msg import Bool


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.get_logger().info("goal_pose_publisher node started")

        self.sub_ = self.create_subscription(Landmarks, 'landmarks', self.publish_goal_pose_, 10)
        self.pub_pose_ = self.create_publisher(PoseStamped, 'goal_pose_right', 10)
        self.pub_claw_closed_ = self.create_publisher(Bool, 'claw_closed', 10)

        self.sub_

    def publish_goal_pose_(self, msg):
        limit = msg.shoulder_bottom_right.y - msg.shoulder_top_right.y

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'goal_pose_frame'
        goal_pose.pose.position.x = (msg.wrist_right.x - msg.shoulder_top_right.x) / limit
        goal_pose.pose.position.y = (msg.wrist_right.y - msg.shoulder_top_right.y) / limit
        goal_pose.pose.position.z = 0.0

        hand_dx = ((msg.hand_right_index.x + msg.hand_right_pinky.x)/2) - msg.wrist_right.x
        hand_dy = ((msg.hand_right_index.y + msg.hand_right_pinky.y)/2) - msg.wrist_right.y

        yaw = math.atan2(hand_dy, hand_dx)

        q = quaternion_from_euler(0, 0, yaw)

        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        claw_closed = Bool()
        claw_closed.data = False

        self.pub_pose_.publish(goal_pose)
        self.pub_claw_closed_.publish(claw_closed)


def main(args=None):
    rclpy.init(args=args)
    goal_pose_publisher = GoalPosePublisher()
    rclpy.spin(goal_pose_publisher)

    goal_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()