import rclpy

from geometry_msgs.msg import Quaternion
from math import sin, cos
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.declare_parameter('pos', 0.)
        pos = self.get_parameter('pos').get_parameter_value().double_value

        loop_rate = self.create_rate(30)

        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'combined_base'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['left_panda_joint1', 'left_panda_joint2', 'left_panda_joint3', 
                                    'left_panda_joint4', 'left_panda_joint5', 'left_panda_joint6', 
                                    'left_panda_joint7', "left_panda_finger_joint1", "left_panda_finger_joint2", 
                                    'right_panda_joint1', 'right_panda_joint2', 'right_panda_joint3', 
                                    'right_panda_joint4', 'right_panda_joint5', 'right_panda_joint6', 
                                    'right_panda_joint7', "right_panda_finger_joint1", "right_panda_finger_joint1"]
                joint_state.position = [0., 0., 0., 0., 0., 0., 0., 0., 0., 
                                        0., 0., 0., 0., 0., 0., 0., 0., 0.]

                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = pos
                odom_trans.transform.translation.y = 0.
                odom_trans.transform.translation.z = 0.
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, 0)

                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()



if __name__ == '__main__':
    main()