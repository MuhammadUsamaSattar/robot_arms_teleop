import os
import time

import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from launch_ros.substitutions import FindPackageShare
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from robot_arms_teleop_interfaces.msg import Landmarks, CombinedLandmarks
from sensor_msgs.msg import Image


class LandMarkPublisher(Node):
    def __init__(self):
        super().__init__('landmark_publisher')
        self.get_logger().info("landmark_publisher node started")

        self.declare_parameter('video_file', 'Video.mp4', ParameterDescriptor(description='Name of video file placed in video directory. ' \
        'Defaults to Video.mp4. Only works with \'VIDEO\' mode.'))
        self.video_file = self.get_parameter('video_file').get_parameter_value().string_value

        self.declare_parameter('mode', 'LIVE_STREAM', ParameterDescriptor(description='Mode of video stream. Options are \'VIDEO\' and \'LIVE_STREAM\''))
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.landmarked_image = []
        self.cv2_bridge = CvBridge()

        self.landmarks_publisher_ = self.create_publisher(CombinedLandmarks, 'combined_landmarks', 10)
        self.image_publisher_ = self.create_publisher(Image, 'image', 1)
        self.init_pose_detector_()
        self.timer_ = self.create_timer(1/self.fps, self.detect_pose_)

    def detect_pose_(self):
        if self.cap.isOpened():
            if self.mode == 'VIDEO':
                frames_passed = max(1, int((((time.time()*1000) - self.frame_time_stamp)*self.fps)/1000) - 1)

                while frames_passed > 0:
                    ret, frame = self.cap.read()
                    frames_passed -= 1
            
                if ret == True:
                    mp_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

                    detection_result = self.landmarker.detect_for_video(mp_frame, self.frame_time_stamp)
                    self.publish_landmark_msgs_(detection_result, mp_frame)

                    self.frame_time_stamp = int((time.time()*1000))

                else:
                    self.get_logger().warn("Video stream not available")
                    self.get_logger().info("Reloading video")
                    pkg_dir = FindPackageShare('goal_pose_publisher').find('goal_pose_publisher')
                    self.cap = cv2.VideoCapture(os.path.join(pkg_dir, 'video', self.video_file))

            elif self.mode == 'LIVE_STREAM':
                ret, frame = self.cap.read()

                if ret == True:
                    mp_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

                    self.landmarker.detect_async(mp_frame, self.frame_time_stamp)

                    while len(self.landmarked_image) == 0:
                        pass

                    self.frame_time_stamp = int((time.time()*1000))
            
                else:
                    self.get_logger().warn("Camera stream not available")
                    self.get_logger().info("Restarting camera stream")
                    self.cap = cv2.VideoCapture(0)

        else:
            self.get_logger().error("Error opening video/steam")

    def init_pose_detector_(self):
        BaseOptions = mp.tasks.BaseOptions
        PoseLandmarker = mp.tasks.vision.PoseLandmarker
        PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        pkg_dir = FindPackageShare('goal_pose_publisher').find('goal_pose_publisher')
        model_path = os.path.join(pkg_dir, 'models', 'pose_landmarker_full.task')

        if self.mode == 'VIDEO':
            options = PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                running_mode=VisionRunningMode.VIDEO,
                )
            self.cap = cv2.VideoCapture(os.path.join(pkg_dir, 'video', self.video_file))
            
        elif self.mode == 'LIVE_STREAM':
            options = PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                running_mode=VisionRunningMode.LIVE_STREAM,
                result_callback=self.publish_landmark_msgs_,
                )
            self.cap = cv2.VideoCapture(0)

            ret, _ = self.cap.read()
            if not ret:
                self.get_logger().error("No camera accessible")

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_time_ms = int(1000/self.fps)

        self.landmarker = PoseLandmarker.create_from_options(options)

        self.frame_time_stamp = int(time.time()*1000)

    def publish_landmark_msgs_(self, *args):
        detection_result = args[0]
        rgb_image = args[1].numpy_view()

        try:
            self.generate_landmarked_image_(detection_result, rgb_image)

            pose_landmarks_list = args[0].pose_landmarks
            msg = CombinedLandmarks()
            msg.right = Landmarks()
            msg.left = Landmarks()

            msg.right.shoulder_top = self.get_positions_from_landmark_list_(pose_landmarks_list, 12)
            msg.right.shoulder_bottom = self.get_positions_from_landmark_list_(pose_landmarks_list, 24)
            msg.right.wrist = self.get_positions_from_landmark_list_(pose_landmarks_list, 16)
            msg.right.thumb = self.get_positions_from_landmark_list_(pose_landmarks_list, 22)
            msg.right.index = self.get_positions_from_landmark_list_(pose_landmarks_list, 20)
            msg.right.pinky = self.get_positions_from_landmark_list_(pose_landmarks_list, 18)

            msg.left.shoulder_top = self.get_positions_from_landmark_list_(pose_landmarks_list, 11)
            msg.left.shoulder_bottom = self.get_positions_from_landmark_list_(pose_landmarks_list, 23)
            msg.left.wrist = self.get_positions_from_landmark_list_(pose_landmarks_list, 15)
            msg.left.thumb = self.get_positions_from_landmark_list_(pose_landmarks_list, 21)
            msg.left.index = self.get_positions_from_landmark_list_(pose_landmarks_list, 19)
            msg.left.pinky = self.get_positions_from_landmark_list_(pose_landmarks_list, 17)

            self.landmarks_publisher_.publish(msg)
            self.image_publisher_.publish(self.cv2_bridge.cv2_to_imgmsg(self.landmarked_image))

        except Exception:
            self.get_logger().warn("Some critical landmarks are not detected in the frame. " \
            "No pose will be published. Image without landmarks will be published.")
            self.image_publisher_.publish(self.cv2_bridge.cv2_to_imgmsg(rgb_image))
    
    def generate_landmarked_image_(self, detection_result, rgb_image):
        pose_landmarks_list = detection_result.pose_landmarks
        landmarked_frame = np.copy(rgb_image)

        # Loop through the detected poses to visualize.
        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]

            # Draw the pose landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
                landmarked_frame,
                pose_landmarks_proto,
                solutions.pose.POSE_CONNECTIONS,
                solutions.drawing_styles.get_default_pose_landmarks_style())
            
        self.landmarked_image = landmarked_frame

    def get_positions_from_landmark_list_(self, landmarks, n):
        msg = Vector3()

        msg.x = landmarks[0][n].x
        msg.y = landmarks[0][n].y
        msg.z = landmarks[0][n].z
        return msg

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()

        super().destroy_node


def main(args=None):
    rclpy.init(args=args)
    landmark_publisher = LandMarkPublisher()
    rclpy.spin(landmark_publisher)

    landmark_publisher.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()