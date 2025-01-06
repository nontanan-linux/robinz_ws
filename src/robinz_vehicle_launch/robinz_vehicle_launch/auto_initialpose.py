#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.running = True
        self.init_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.publish_initial_pose_once)

    def publish_initial_pose_once(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.init_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        if elapsed_time <= 3.0:  # Publish within the first 3 seconds
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.pose.pose.position.x = -1.9795588942562359  # Set the initial X position
            initial_pose.pose.pose.position.y = -0.4501933727001997  # Set the initial Y position
            initial_pose.pose.pose.position.z = 0.0                 # Set the initial Z position
            initial_pose.pose.pose.orientation.z = -0.003619069291273  # Set orientation
            initial_pose.pose.pose.orientation.w = 0.9999934511472888  # Set orientation

            # Covariance (optional, can be tuned as needed)
            # initial_pose.pose.covariance = [0.1, 0, 0, 0, 0, 0,
            #                                 0, 0.1, 0, 0, 0, 0,
            #                                 0, 0, 0.1, 0, 0, 0,
            #                                 0, 0, 0, 0.1, 0, 0,
            #                                 0, 0, 0, 0, 0.1, 0,
            #                                 0, 0, 0, 0, 0, 0.1]

            self.publisher_.publish(initial_pose)
            self.get_logger().info("Published initial pose.")
        else:
            # Stop the timer after 3 seconds
            self.timer.cancel()
            self.get_logger().info("Timer stopped after 3 seconds.")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
