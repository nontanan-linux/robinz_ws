#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(2.0, self.publish_initial_pose)

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = -1.9795588942562359     # Set the initial X position
        initial_pose.pose.pose.position.y = -0.4501933727001997     # Set the initial Y position
        initial_pose.pose.pose.position.z = 0.0                     # Set the initial Z position
        initial_pose.pose.pose.orientation.z = -0.003619069291273   # Set orientation
        initial_pose.pose.pose.orientation.w = 0.9999934511472888   # Set orientation

        # Covariance (optional, can be tuned as needed)
        # initial_pose.pose.covariance = [0.1, 0, 0, 0, 0, 0,
        #                                 0, 0.1, 0, 0, 0, 0,
        #                                 0, 0, 0.1, 0, 0, 0,
        #                                 0, 0, 0, 0.1, 0, 0,
        #                                 0, 0, 0, 0, 0.1, 0,
        #                                 0, 0, 0, 0, 0, 0.1]

        self.publisher_.publish(initial_pose)
        self.get_logger().info("Published initial pose.")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
