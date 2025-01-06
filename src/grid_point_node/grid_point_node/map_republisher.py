#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        
        # Define a QoS profile with transient local durability
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            qos_profile  # Use the same QoS profile for subscription
        )
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.latest_map = None

    def listener_callback(self, msg):
        self.latest_map = msg

    def timer_callback(self):
        if self.latest_map is not None:
            self.publisher.publish(self.latest_map)
            self.get_logger().info('Re-published the latest map data')

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
