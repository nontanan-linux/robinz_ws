#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math

class PathRecorderNode(Node):

    def __init__(self):
        super().__init__('path_recorder_node')

        # Parameters
        self.csv_file_path = self.declare_parameter('csv_file_path', '/home/nontanan/robinz_ws/src/robinz_vehicle_launch/csv/robot_path.csv').get_parameter_value().string_value

        # Subscribe to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Open CSV file for writing
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header of the CSV file
        self.csv_writer.writerow(['heading (radians)', 'x (meters)', 'y (meters)', 'theta (radians)', 'cost'])
        
        self.previous_x = None
        self.previous_y = None

        self.get_logger().info(f'Path recorder started, saving to {self.csv_file_path}')

    def odom_callback(self, msg):
        # Extract position and orientation from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation

        # Compute the heading (yaw) from the quaternion orientation
        heading = self.get_yaw_from_quaternion(orientation)

        # Calculate the movement cost if there is a previous position
        if self.previous_x is not None and self.previous_y is not None:
            dx = x - self.previous_x
            dy = y - self.previous_y
            cost = math.sqrt(dx**2 + dy**2)
        else:
            cost = 0.0

        # Update the previous position
        self.previous_x = x
        self.previous_y = y

        # Write the motion primitive to the CSV file
        self.csv_writer.writerow([heading, x, y, heading, cost])

    def get_yaw_from_quaternion(self, orientation):
        """
        Convert quaternion to yaw (heading).
        """
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Quaternion to Euler angles conversion (yaw)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def destroy_node(self):
        # Close the CSV file when the node is destroyed
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # Create the PathRecorderNode
    node = PathRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
