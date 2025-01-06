import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import yaml

class StereoDepthEstimator(Node):
    def __init__(self):
        super().__init__('stereo_depth_estimator')
        self.bridge = CvBridge()
        
        # Load camera calibration parameters
        # self.load_camera_calibration('/home/nontanan/robinz_ws/src/robinz_stereo_camera/config/left.yaml',
        #                               '/home/nontanan/robinz_ws/src/robinz_stereo_camera/config/right.yaml')
        
        self.left_calibration = self.load_camera_calibration('/home/nontanan/robinz_ws/src/robinz_stereo_camera/config/left.yaml')
        self.right_calibration = self.load_camera_calibration('/home/nontanan/robinz_ws/src/robinz_stereo_camera/config/right.yaml')

        self.left_sub = self.create_subscription(Image, '/stereo/left/image_raw', self.left_callback, 10)
        self.right_sub = self.create_subscription(Image, '/stereo/right/image_raw', self.right_callback, 10)

        self.left_image = None
        self.right_image = None

    def load_camera_calibration(self, yaml_path):
        with open(yaml_path, 'r') as config:
            left_calib = yaml.safe_load(left_file)
        
        with open(right_yaml_path, 'r') as right_file:
            right_calib = yaml.safe_load(right_file)
        
        # Extracting camera parameters from the left camera calibration file
        self.focal_length_x = left_calib['camera_matrix']['data'][0]  # 1st element of camera matrix
        self.focal_length_y = left_calib['camera_matrix']['data'][4]  # 5th element of camera matrix
        self.cx = left_calib['camera_matrix']['data'][2]              # 3rd element of camera matrix
        self.cy = left_calib['camera_matrix']['data'][5]              # 6th element of camera matrix

        # Define the projection matrix for stereo depth estimation
        self.projection_matrix = np.array([[left_calib['projection_matrix']['data'][0], 0, left_calib['projection_matrix']['data'][2], 0],
                                           [0, left_calib['projection_matrix']['data'][5], left_calib['projection_matrix']['data'][6], 0],
                                           [0, 0, 1, 0]])

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()

    def process_images(self):
        if self.left_image is not None and self.right_image is not None:
            # Convert images to grayscale
            gray_left = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)

            # Stereo matching using OpenCV
            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            disparity = stereo.compute(gray_left, gray_right)

            # Ensure disparity is in 16-bit signed format for depth calculation
            disparity = np.int16(disparity)

            # Convert disparity to depth map (using projection matrix)
            depth_map = cv2.reprojectImageTo3D(disparity, self.projection_matrix)

            # Display or publish the depth map
            cv2.imshow('Depth Map', depth_map)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthEstimator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
