import math

import rclpy
import numpy as np
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from robot_arms_teleop_interfaces.msg import CombinedLandmarks, CombinedPoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def distance(p1, p2):
    # Calculates euclidean distance between 2D p1 and p2
    return math.sqrt(((p2[0]-p1[0])**2) + ((p2[1]-p1[1])**2))

def quaternion_from_euler(ai, aj, ak):
    # Generates quaternion from euler angles
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

        # Generates transforms for left and right side
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        while True:
            try:
                left_transform = self.tf_buffer.lookup_transform('combined_base', 'left_goal_pose_frame', 
                                                                 rclpy.time.Time())
                right_transform = self.tf_buffer.lookup_transform('combined_base', 'right_goal_pose_frame', 
                                                                 rclpy.time.Time())
                self.transform = [left_transform, right_transform]

                break

            except Exception:
                rclpy.spin_once(self)

        self.sub_ = self.create_subscription(CombinedLandmarks, 'combined_landmarks', self.publish_goal_pose_, 10)
        self.pub_pose_ = self.create_publisher(CombinedPoseStamped, 'combined_goal_pose', 10)
        self.left_pub_pose_ = self.create_publisher(PoseStamped, 'left_goal_pose', 10)
        self.right_pub_pose_ = self.create_publisher(PoseStamped, 'right_goal_pose', 10)

        self.buffer_size_ = 5 # Size of buffer
        self.goal_pose_buffer_ = [] # Stores a buffer of goal poses
        self.jaw_sides_ = [False, False] # Stores values of last jaw state
        self.jaw_closed_conf_ = 0.8 # Confidence for opening/closing the jaw
        self.scale_ = 0.2 # Scale of movement of goal pose around the goal pose frame
        self.sub_

    def publish_goal_pose_(self, msg):
        goal_pose = CombinedPoseStamped()
        goal_pose_sides = [goal_pose.left, goal_pose.right]
        names = ["left", "right"]
        self.goal_pose_buffer_.append([])

        # Determines the goal pose for both sides using landmarks
        for side in [msg.left, msg.right]:
            limit = side.shoulder_bottom.y - side.shoulder_top.y # Calculates the value of scale according to height difference between top and bottom of torso
            goal = []
            goal.append((side.wrist.x - side.shoulder_top.x) / limit) # Appends dx
            goal.append(-((side.wrist.y - side.shoulder_top.y) / limit)) # Appends dy

            hand_dx = ((side.index.x + side.pinky.x)/2) - side.wrist.x
            hand_dy = -(((side.index.y + side.pinky.y)/2) - side.wrist.y)

            yaw = math.atan2(hand_dy, hand_dx)
            goal.append(yaw) # Appends the hand rotation

            # Hand is closed if the average distance of thumb, index and pinky to the wrist falls below
            # a certain value
            closeness_metric = (distance([side.thumb.x, side.thumb.y], [side.wrist.x, side.wrist.y]) + 
                                distance([side.index.x, side.index.y], [side.wrist.x, side.wrist.y]) + 
                                distance([side.pinky.x, side.pinky.y], [side.wrist.x, side.wrist.y]))/3
            if closeness_metric > 0.05:
                goal.append(False)
            else:
                goal.append(True)

            self.goal_pose_buffer_[-1].append(goal)

        if len(self.goal_pose_buffer_) == self.buffer_size_:
            # Calculates the pose for both sides
            for i in range(len(goal_pose_sides)):
                # Generates the pose message according to the goal frame
                goal_pose_sides[i].header.stamp = self.get_clock().now().to_msg()
                goal_pose_sides[i].header.frame_id = names[i] + '_goal_pose_frame'
                goal_pose_sides[i].pose.position.x = 0.
                goal_pose_sides[i].pose.position.y = 0.
                goal_pose_sides[i].pose.position.z = 0.
                yaw = 0.

                for goal in self.goal_pose_buffer_:
                    goal_pose_sides[i].pose.position.x += self.scale_*goal[i][0]/self.buffer_size_
                    goal_pose_sides[i].pose.position.y += self.scale_*goal[i][1]/self.buffer_size_
                    yaw += goal[i][2]/self.buffer_size_

                q = quaternion_from_euler(0, 0, yaw)

                goal_pose_sides[i].pose.orientation.x = q[0]
                goal_pose_sides[i].pose.orientation.y = q[1]
                goal_pose_sides[i].pose.orientation.z = q[2]
                goal_pose_sides[i].pose.orientation.w = q[3]

                # Converts pose in goal frame to combined frame
                goal_pose_sides[i] = tf2_geometry_msgs.do_transform_pose_stamped(goal_pose_sides[i], self.transform[i])

                # Assigns a new value to jaw closing variable if the confidence exceed a certain value
                jaw_closed_list = [goal[i][3] for goal in self.goal_pose_buffer_]
                if jaw_closed_list.count(True)/self.buffer_size_ >= self.jaw_closed_conf_:
                    self.jaw_sides_[i] = True

                elif jaw_closed_list.count(False)/self.buffer_size_ >= self.jaw_closed_conf_:
                    self.jaw_sides_[i] = False

            # Assigns the calculated poses and jaw state to the msg
            goal_pose.left = goal_pose_sides[0]
            goal_pose.right = goal_pose_sides[1]
            goal_pose.left_jaw_closed = self.jaw_sides_[0]
            goal_pose.right_jaw_closed = self.jaw_sides_[1]

            # Publishes the combined and individual goal poses and removes the first element from buffer
            self.pub_pose_.publish(goal_pose)
            self.left_pub_pose_.publish(goal_pose.left)
            self.right_pub_pose_.publish(goal_pose.right)
            self.goal_pose_buffer_.pop(0)


def main(args=None):
    rclpy.init(args=args)
    goal_pose_publisher = GoalPosePublisher()
    rclpy.spin(goal_pose_publisher)

    goal_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()