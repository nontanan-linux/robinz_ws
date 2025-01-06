import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WaypointCreator(Node):
    def __init__(self):
        super().__init__('waypoint_creator')
        self.waypoint_pub = self.create_publisher(Marker, 'waypoints', 10)

        # Create a Marker to visualize waypoints
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.SPHERE
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0  # Red
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        # Subscribe to the RViz Click topic
        self.create_subscription(Point, '/clicked_point', self.point_callback, 10)

    def point_callback(self, msg):
        self.get_logger().info(f"Point clicked: {msg.x}, {msg.y}, {msg.z}")

        # Update the marker position to the clicked point
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.pose.position.x = msg.x
        self.marker.pose.position.y = msg.y
        self.marker.pose.position.z = msg.z

        # Publish the marker
        self.marker.id += 1  # Change ID to show new marker
        self.waypoint_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    waypoint_creator = WaypointCreator()
    rclpy.spin(waypoint_creator)
    waypoint_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
