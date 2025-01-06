import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import yaml
import numpy as np

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')
        self.declare_parameter('calibration_file', '/home/nontanan/robinz_ws/src/robinz_vehicle_launch/config/left.yaml')
        self.declare_parameter('video_device', '/dev/video2')
        self.declare_parameter('namespace', '/stereo/left')

        # Get parameters
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.video_device = self.get_parameter('video_device').get_parameter_value().string_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        self.get_logger().info(f'Using calibration file: {self.calibration_file}')
        self.get_logger().info(f'Using video device: {self.video_device}')
        self.get_logger().info(f'Using namespace: {self.namespace}')

        # Load camera calibration parameters from YAML file
        self.load_camera_calibration(self.calibration_file)

        # Create OpenCV VideoCapture
        self.cap = cv2.VideoCapture(self.video_device)

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open video device.")
            return

        # Create publishers
        self.image_publisher = self.create_publisher(Image, f'{self.namespace}/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, f'{self.namespace}/camera_info', 10)
        
        # Initialize CvBridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Start capturing frames
        self.capture_frames()

    def load_camera_calibration(self, file_path):
        with open(file_path, 'r') as file:
            calibration_data = yaml.safe_load(file)
            self.get_logger().info(f"Loaded calibration data: {calibration_data}")

            # Store calibration parameters
            self.camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape((3, 3))
            self.distortion_coefficients = np.array(calibration_data['distortion_coefficients']['data'])
            self.rectification_matrix = np.array(calibration_data['rectification_matrix']['data']).reshape((3, 3))
            self.projection_matrix = np.array(calibration_data['projection_matrix']['data']).reshape((3, 4))

            # Extract image dimensions
            self.image_width = calibration_data['image_width']
            self.image_height = calibration_data['image_height']
            self.get_logger().info(f"Image size: {self.image_width}x{self.image_height}")

            # Setup CameraInfo message
            self.camera_info_msg = CameraInfo()
            self.camera_info_msg.width = self.image_width
            self.camera_info_msg.height = self.image_height
            self.camera_info_msg.distortion_model = calibration_data['distortion_model']
            self.camera_info_msg.D = self.distortion_coefficients.tolist()  # Corrected this line
            self.camera_info_msg.K = self.camera_matrix.flatten().tolist()
            self.camera_info_msg.R = self.rectification_matrix.flatten().tolist()
            self.camera_info_msg.P = self.projection_matrix.flatten().tolist()
            self.camera_info_msg.header.frame_id = f'{self.namespace}/camera_frame'

    def capture_frames(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error: Could not read frame.")
                break
            
            # Undistort the frame using the calibration data
            frame = self.undistort_frame(frame)

            # Convert the frame to a ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the Image and CameraInfo messages
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = f'{self.namespace}/camera_frame'
            self.image_publisher.publish(image_msg)
            self.camera_info_msg.header.stamp = image_msg.header.stamp
            self.camera_info_publisher.publish(self.camera_info_msg)

            # Display the frame (optional)
            cv2.imshow('Stereo Camera Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def undistort_frame(self, frame):
        if hasattr(self, 'camera_matrix') and hasattr(self, 'distortion_coefficients'):
            undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients)
            return undistorted_frame
        return frame

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
