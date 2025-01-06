#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.path_publisher = self.create_publisher(Path, 'path_segment', 10)
        self.node_publisher = self.create_publisher(Path, 'node', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Publish every 2 seconds
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.node_msg = Path()
        self.node_msg.header.frame_id = "map"
        self.waypoints = self.load_waypoints_from_csv('/home/nontanan/robinz_ws/src/path_tracking/csv/waypoint.csv')  # Load waypoints from CSV
        self.nodes = self.load_node_from_csv('/home/nontanan/robinz_ws/src/robinz_vehicle_launch/csv/node.csv')

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
    
    def load_node_from_csv(self, csv_file):
        nodes = []
        with open(csv_file, mode='r') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                x = float(row['x'])
                y = float(row['y'])
                nodes.append([x, y])
        return nodes

    def timer_callback(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses = []
        self.node_msg.header.stamp = self.get_clock().now().to_msg()
        self.node_msg.poses = []

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
        
        for node in self.nodes:
            station = PoseStamped()
            station.header.frame_id = 'map'
            station.header.stamp = self.get_clock().now().to_msg()
            station.pose.position.x = node[0]
            station.pose.position.y = node[1]
            self.node_msg.poses.append(station)

        # Publish the path
        self.path_publisher.publish(self.path_msg)
        self.node_publisher.publish(self.node_msg)
        self.get_logger().info(f'Publishing path with {len(self.path_msg.poses)} waypoints.')

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
