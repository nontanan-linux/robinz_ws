#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import csv

class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')
        self.path_publisher = self.create_publisher(Path, 'recorded_path', 10)
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.path.header.stamp = self.get_clock().now().to_msg()
        
        self.csv_file = open('/tmp/recorded_path.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.save_to_csv(pose.pose)
        self.publish_path()

    def save_to_csv(self, pose):
        self.csv_writer.writerow([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)
        self.get_logger().info('Publishing recorded path')

    def __del__(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
