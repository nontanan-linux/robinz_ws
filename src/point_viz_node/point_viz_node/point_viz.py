#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from rclpy.qos import QoSProfile

class PointVizNode(Node):
    def __init__(self):
        super().__init__('point_viz_node')

        # Publisher for marker array
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', QoSProfile(depth=10))
        
        # Create a timer to publish markers
        self.timer = self.create_timer(0.1, self.timer_callback)

        # To store points
        self.points = []

    def timer_callback(self):
        if self.points:
            marker_array = MarkerArray()
            for i, point in enumerate(self.points):
                marker = Marker()
                marker.header = Header()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "points"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = point
                marker.scale.x = 0.1  # Sphere size
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0  # Alpha
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)

            self.marker_pub.publish(marker_array)

    def add_point(self, x, y, z):
        point = Point(x=x, y=y, z=z)
        self.points.append(point)

def main(args=None):
    rclpy.init(args=args)
    node = PointVizNode()

    # Simulate point addition on clicks (replace this with actual click handling)
    node.add_point(1.0, 1.0, 0.0)
    node.add_point(2.0, 2.0, 0.0)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
