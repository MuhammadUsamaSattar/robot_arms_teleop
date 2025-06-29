import math

import rclpy
import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_arms_teleop_interfaces.msg import Landmarks
from std_msgs.msg import Bool


def distance(p1, p2):
    return math.sqrt(((p2[0]-p1[0])**2) + ((p2[1]-p1[1])**2))

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
        self.get_logger().info(self.get_namespace() + "/goal_pose_publisher node started")

        self.sub_ = self.create_subscription(Landmarks, 'landmarks', self.publish_goal_pose_, 10)
        self.pub_pose_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.pub_claw_closed_ = self.create_publisher(Bool, 'claw_closed', 10)

        self.sub_
        self.goal_pose_buffer = []
        self.claw_closed_buffer = []
        self.claw_closed = Bool()
        self.claw_closed_conf = 0.8
        self.buffer_size = 5

    def publish_goal_pose_(self, msg):
        limit = msg.shoulder_bottom.y - msg.shoulder_top.y

        goal_pose = PoseStamped()
        goal = []
        goal.append((msg.wrist.x - msg.shoulder_top.x) / limit)
        goal.append((msg.wrist.y - msg.shoulder_top.y) / limit)

        hand_dx = ((msg.index.x + msg.pinky.x)/2) - msg.wrist.x
        hand_dy = ((msg.index.y + msg.pinky.y)/2) - msg.wrist.y

        yaw = math.atan2(hand_dy, hand_dx)
        goal.append(yaw)

        self.goal_pose_buffer.append(goal)

        closeness_metric = (distance([msg.thumb.x, msg.thumb.y], [msg.wrist.x, msg.wrist.y]) + 
                            distance([msg.index.x, msg.index.y], [msg.wrist.x, msg.wrist.y]) + 
                            distance([msg.pinky.x, msg.pinky.y], [msg.wrist.x, msg.wrist.y]))/3
        if closeness_metric > 0.05:
            self.claw_closed_buffer.append(False)
        else:
            self.claw_closed_buffer.append(True)

        if len(self.goal_pose_buffer) == self.buffer_size:
            goal_pose.pose.position.x = 0.
            goal_pose.pose.position.y = 0.
            goal_pose.pose.position.z = 0.
            yaw = 0.

            for goal in self.goal_pose_buffer:
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.header.frame_id = self.get_namespace() + '_goal_pose_frame'

                goal_pose.pose.position.x += goal[0]/self.buffer_size
                goal_pose.pose.position.y += goal[1]/self.buffer_size
                yaw += goal[2]/self.buffer_size

            q = quaternion_from_euler(0, 0, yaw)

            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]

            if self.claw_closed_buffer.count(True)/self.buffer_size >= self.claw_closed_conf:
                self.claw_closed.data = True

            elif self.claw_closed_buffer.count(False)/self.buffer_size >= self.claw_closed_conf:
                self.claw_closed.data = False

            self.goal_pose_buffer.pop(0)
            self.pub_pose_.publish(goal_pose)
            self.claw_closed_buffer.pop(0)
            self.pub_claw_closed_.publish(self.claw_closed)


def main(args=None):
    rclpy.init(args=args)
    goal_pose_publisher = GoalPosePublisher()
    rclpy.spin(goal_pose_publisher)

    goal_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()