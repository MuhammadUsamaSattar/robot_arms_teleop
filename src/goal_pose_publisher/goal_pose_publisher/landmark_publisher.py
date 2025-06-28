import mediapipe as mp
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
from launch_ros.substitutions import FindPackageShare
import os
import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class LandMarkPublisher(Node):
    def __init__(self):
        super().__init__('landmark_publisher')
        self.get_logger().info("landmark_publisher node started")
        self.landmarked_image = []

        self.publisher_ = self.create_publisher(Pose, 'landmarks', 10)
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
                    self.draw_landmarks_on_image_(detection_result, mp_frame)

                    cv2.imshow("Landmark Detection", self.landmarked_image)
                    cv2.waitKey(1)

                    self.frame_time_stamp = int((time.time()*1000))

                else:
                    self.get_logger().warn("No frame could be read from the video/stream")

            elif self.mode == 'LIVE_STREAM':
                ret, frame = self.cap.read()

                if ret == True:
                    mp_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

                    self.landmarker.detect_async(mp_frame, self.frame_time_stamp)

                    while len(self.landmarked_image) == 0:
                        pass

                    cv2.imshow("Landmark Detection", self.landmarked_image)
                    cv2.waitKey(1)

                    self.frame_time_stamp = int((time.time()*1000))
            
                else:
                    self.get_logger().warn("No frame could be read from the video/stream")

        else:
            self.get_logger().error("Error opening video/steam")

    def init_pose_detector_(self):
        self.mode = 'VIDEO'

        BaseOptions = mp.tasks.BaseOptions
        PoseLandmarker = mp.tasks.vision.PoseLandmarker
        PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        #pkg_dir = ''
        pkg_dir = FindPackageShare('goal_pose_publisher').find('goal_pose_publisher')
        model_path = os.path.join(pkg_dir, 'models', 'pose_landmarker_full.task')

        if self.mode == 'VIDEO':
            options = PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                running_mode=VisionRunningMode.VIDEO,
                )
            self.cap = cv2.VideoCapture(os.path.join(pkg_dir, 'video', 'Video.mp4'))
            
        elif self.mode == 'LIVE_STREAM':
            options = PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                running_mode=VisionRunningMode.LIVE_STREAM,
                result_callback=self.draw_landmarks_on_image_,
                )
            self.cap = cv2.VideoCapture(0)

        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_time_ms = int(1000/self.fps)

        self.landmarker = PoseLandmarker.create_from_options(options)

        self.frame_time_stamp = int(time.time()*1000)
    
    def draw_landmarks_on_image_(self, *args):
        detection_result = args[0]
        rgb_image = args[1].numpy_view()
        
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