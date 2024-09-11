#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(Path, 'path', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Publish every 2 seconds
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.waypoints = self.load_waypoints_from_csv('/home/nontanan/robinz_ws/src/path_tracking/csv/waypoint.csv')  # Load waypoints from CSV

    def load_waypoints_from_csv(self, csv_file):
        waypoints = []
        with open(csv_file, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                waypoints.append([x, y, z])
        return waypoints

    def timer_callback(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses = []

        # Convert CSV waypoints to PoseStamped and fill Path message
        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.w = 1.0  # No rotation
            self.path_msg.poses.append(pose)

        # Publish the path
        self.publisher_.publish(self.path_msg)
        self.get_logger().info(f'Publishing path with {len(self.path_msg.poses)} waypoints.')

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
