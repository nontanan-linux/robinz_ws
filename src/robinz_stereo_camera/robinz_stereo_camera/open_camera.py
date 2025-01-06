import cv2
import yaml
import numpy as np

class StereoCamera:
    def __init__(self, left_calibration_path, right_calibration_path, left_camera_index=0, right_camera_index=2, baseline=0.1):
        self.left_calibration_path = left_calibration_path
        self.right_calibration_path = right_calibration_path
        self.left_camera_index = left_camera_index
        self.right_camera_index = right_camera_index
        
        self.left_camera = None
        self.right_camera = None
        
        self.left_params = self.load_calibration(left_calibration_path)
        self.right_params = self.load_calibration(right_calibration_path)

        # Extract parameters for left camera
        self.left_camera_matrix = np.array(self.left_params['camera_matrix']['data']).reshape(3, 3)
        self.left_distortion_coeffs = np.array(self.left_params['distortion_coefficients']['data'])
        
        # Extract parameters for right camera
        self.right_camera_matrix = np.array(self.right_params['camera_matrix']['data']).reshape(3, 3)
        self.right_distortion_coeffs = np.array(self.right_params['distortion_coefficients']['data'])

        # Set baseline distance (in meters)
        self.baseline = baseline
        
        # Create StereoBM object for disparity computation
        self.stereo_bm = cv2.StereoBM_create(numDisparities=16, blockSize=15)

        # Define the rectification transformation
        self.rectify_left, self.rectify_right = self.create_rectification_maps()

    def load_calibration(self, file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data

    def create_rectification_maps(self):
        """Create the rectification maps for both cameras."""
        # Left camera rectification
        R_left = np.eye(3)  # Identity matrix for left camera (no rotation)
        map1_left, map2_left = cv2.initUndistortRectifyMap(
            self.left_camera_matrix, self.left_distortion_coeffs, 
            R_left, self.left_camera_matrix, 
            (640, 480), cv2.CV_32FC1
        )
        
        # Right camera rectification (assuming a similar structure)
        R_right = np.eye(3)  # Identity matrix for right camera (no rotation)
        map1_right, map2_right = cv2.initUndistortRectifyMap(
            self.right_camera_matrix, self.right_distortion_coeffs, 
            R_right, self.right_camera_matrix, 
            (640, 480), cv2.CV_32FC1
        )

        return (map1_left, map2_left), (map1_right, map2_right)

    def open_cameras(self):
        """Open the left and right cameras."""
        self.left_camera = cv2.VideoCapture(self.left_camera_index)
        self.right_camera = cv2.VideoCapture(self.right_camera_index)
        
        if not self.left_camera.isOpened() or not self.right_camera.isOpened():
            print("Error: Could not open one or both cameras.")
            return False
        return True
    
    def calculate_depth(self, disparity):
        """Calculate depth from disparity."""
        # Avoid division by zero
        disparity[disparity == 0] = 0.1  # Small value to prevent division by zero
        depth = (self.left_camera_matrix[0, 0] * self.baseline) / disparity
        return depth

    def capture_frames(self):
        """Capture and process frames from both cameras."""
        while True:
            ret_left, frame_left = self.left_camera.read()
            ret_right, frame_right = self.right_camera.read()

            if not ret_left or not ret_right:
                print("Error: Could not read frames from cameras.")
                break

            # Undistort and rectify the images
            left_rectified = cv2.remap(frame_left, self.rectify_left[0], self.rectify_left[1], cv2.INTER_LINEAR)
            right_rectified = cv2.remap(frame_right, self.rectify_right[0], self.rectify_right[1], cv2.INTER_LINEAR)

            # Convert to grayscale
            gray_left = cv2.cvtColor(left_rectified, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_rectified, cv2.COLOR_BGR2GRAY)

            # Calculate disparity
            disparity = self.stereo_bm.compute(gray_left, gray_right).astype(np.float32) / 16.0

            # Calculate depth
            depth = self.calculate_depth(disparity)

            # Display the images and results
            cv2.imshow('Left Camera', frame_left)
            cv2.imshow('Right Camera', frame_right)
            cv2.imshow('Disparity', disparity / disparity.max())  # Normalize for display
            cv2.imshow('Depth', depth / depth.max())  # Normalize for display

            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    def release_cameras(self):
        """Release the camera resources."""
        if self.left_camera:
            self.left_camera.release()
        if self.right_camera:
            self.right_camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Paths to the calibration files
    left_calibration_path = '/home/nontanan/robinz_ws/src/robinz_stereo_camera/config/left.yaml'
    right_calibration_path = '/home/nontanan/robinz_ws/src/robinz_stereo_camera/config/right.yaml'
    
    # Define the baseline (you need to measure this distance)
    baseline = 0.1  # Example: 10 cm, adjust this value based on your setup

    stereo_camera = StereoCamera(left_calibration_path, right_calibration_path, baseline=baseline)

    if stereo_camera.open_cameras():
        stereo_camera.capture_frames()
    
    stereo_camera.release_cameras()
