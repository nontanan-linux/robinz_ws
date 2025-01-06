# robinz_stereo_camera/stereo_camera_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import yaml
import os

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # Parameters
        self.declare_parameter('video_device_left', '/dev/video0')
        self.declare_parameter('video_device_right', '/dev/video1')
        self.declare_parameter('calibration_file_left', '/home/nontanan/robinz_ws/src/robinz_stereo_camera/config/left.yaml')
        self.declare_parameter('calibration_file_right', '/home/nontanan/robinz_ws/src/robinz_stereo_camera/config/right.yaml')
        self.declare_parameter('namespace', '/stereo')

        self.video_device_left = self.get_parameter('video_device_left').get_parameter_value().string_value
        self.video_device_right = self.get_parameter('video_device_right').get_parameter_value().string_value
        self.calibration_file_left = self.get_parameter('calibration_file_left').get_parameter_value().string_value
        self.calibration_file_right = self.get_parameter('calibration_file_right').get_parameter_value().string_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # Camera setup
        self.cap_left = cv2.VideoCapture(self.video_device_left)
        self.cap_right = cv2.VideoCapture(self.video_device_right)

        # Publish topics
        self.image_pub_left = self.create_publisher(Image, f'{self.namespace}/left/image_raw', 10)
        self.image_pub_right = self.create_publisher(Image, f'{self.namespace}/right/image_raw', 10)
        self.camera_info_pub_left = self.create_publisher(CameraInfo, f'{self.namespace}/left/camera_info', 10)
        self.camera_info_pub_right = self.create_publisher(CameraInfo, f'{self.namespace}/right/camera_info', 10)
        self.depth_pub = self.create_publisher(Image, f'{self.namespace}/depth', 10)

        self.bridge = CvBridge()
        self.load_camera_calibrations()
        self.create_stereo_matcher()

        self.timer = self.create_timer(0.1, self.publish_images)

    def load_camera_calibrations(self):
        # Load left camera calibration
        with open(self.calibration_file_left, 'r') as file:
            calibration_data_left = yaml.safe_load(file)

            self.image_width_left = calibration_data_left['image_width']
            self.image_height_left = calibration_data_left['image_height']
            self.camera_info_left = CameraInfo()
            self.camera_info_left.width = self.image_width_left
            self.camera_info_left.height = self.image_height_left
            self.camera_info_left.distortion_model = calibration_data_left['distortion_model']
            # self.camera_info_left.D = calibration_data_left['distortion_coefficients']['data']
            # self.camera_info_left.K = calibration_data_left['camera_matrix']['data']
            self.camera_info_left.header.frame_id = f'{self.namespace}/left_camera_frame'

        # Load right camera calibration
        with open(self.calibration_file_right, 'r') as file:
            calibration_data_right = yaml.safe_load(file)

            self.image_width_right = calibration_data_right['image_width']
            self.image_height_right = calibration_data_right['image_height']
            self.camera_info_right = CameraInfo()
            self.camera_info_right.width = self.image_width_right
            self.camera_info_right.height = self.image_height_right
            self.camera_info_right.distortion_model = calibration_data_right['distortion_model']
            # self.camera_info_right.D = calibration_data_right['distortion_coefficients']['data']
            # self.camera_info_right.K = calibration_data_right['camera_matrix']['data']
            self.camera_info_right.header.frame_id = f'{self.namespace}/right_camera_frame'

    def create_stereo_matcher(self):
        # Create a stereo matcher
        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16 * 5,  # Must be divisible by 16
            blockSize=5,
            P1=8 * 3 * 5 ** 2,
            P2=32 * 3 * 5 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM
        )

    def publish_images(self):
        ret_left, frame_left = self.cap_left.read()
        ret_right, frame_right = self.cap_right.read()

        if ret_left and ret_right:
            # Convert the images to ROS format
            img_msg_left = self.bridge.cv2_to_imgmsg(frame_left, encoding='bgr8')
            img_msg_right = self.bridge.cv2_to_imgmsg(frame_right, encoding='bgr8')

            # Set the header
            img_msg_left.header.stamp = self.get_clock().now().to_msg()
            img_msg_right.header.stamp = self.get_clock().now().to_msg()

            # Publish images
            self.image_pub_left.publish(img_msg_left)
            self.image_pub_right.publish(img_msg_right)

            # Publish camera info
            self.camera_info_left.header.stamp = img_msg_left.header.stamp
            self.camera_info_pub_left.publish(self.camera_info_left)

            self.camera_info_right.header.stamp = img_msg_right.header.stamp
            self.camera_info_pub_right.publish(self.camera_info_right)

            # Compute depth
            self.compute_depth(frame_left, frame_right)

    def compute_depth(self, frame_left, frame_right):
        # Convert frames to grayscale
        gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

        # Compute the disparity map
        disparity = self.stereo_matcher.compute(gray_left, gray_right).astype('float32') / 16.0

        # Convert disparity to depth map
        depth_map = cv2.reprojectImageTo3D(disparity, np.eye(4))  # Adjust this as per your camera's calibration
        depth_map_msg = self.bridge.cv2_to_imgmsg(disparity, encoding='32FC1')
        depth_map_msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(depth_map_msg)

    def destroy_node(self):
        self.cap_left.release()
        self.cap_right.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
