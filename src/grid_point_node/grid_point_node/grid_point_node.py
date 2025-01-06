#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import rclpy
import heapq
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class GridMarkerNode(Node):
    def __init__(self):
        super().__init__('grid_marker_node')
        self.get_logger().info("GridMarkerNode initialized")

        # QoS profile for subscription
        qos_profile = QoSProfile(depth=5)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subscriber to the map
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.get_logger().info("Subscription to /map created")

        # Publisher for the grid markers
        self.marker_pub = self.create_publisher(MarkerArray, '/grid_markers', 10)

    def create_grid_array(self, msg):
        """
        Create a 2D array representing the map.
        Black (occupied) is 0, and white (free) is 1.
        """
        grid_array = np.zeros((msg.info.height, msg.info.width), dtype=int)

        for y in range(msg.info.height):
            for x in range(msg.info.width):
                index = y * msg.info.width + x
                # Convert occupancy values to 0 (occupied) and 1 (free)
                if msg.data[index] >= 50:  # Occupied if value is >= 50
                    grid_array[y, x] = 0  # Black (occupied)
                else:
                    grid_array[y, x] = 1  # White (free)
        return grid_array

    def plot_grid_array(self, grid_array, map_resolution, map_origin_x, map_origin_y):
        plt.imshow(grid_array, cmap='gray', origin='lower',
                extent=(map_origin_x, map_origin_x + grid_array.shape[1] * map_resolution,
                        map_origin_y, map_origin_y + grid_array.shape[0] * map_resolution))
        plt.colorbar(label='Occupancy')
        plt.title('Grid Occupancy Map')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.axis('equal')  # Keep the aspect ratio square
        plt.show()

    def map_callback(self, msg):
        self.get_logger().info("Map callback triggered!")
        self.get_logger().info(f"Received map with dimensions: {msg.info.width} x {msg.info.height}")

        # Create the grid array based on the occupancy values
        grid_array = self.create_grid_array(msg)
        self.get_logger().info(f"Grid array created with shape: {grid_array.shape}")

        # Plot the grid array
        self.plot_grid_array(grid_array, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)

        # Extract map information
        map_width = msg.info.width
        map_height = msg.info.height
        map_resolution = msg.info.resolution
        map_origin_x = msg.info.origin.position.x
        map_origin_y = msg.info.origin.position.y

        # Create MarkerArray for grid points
        marker_array = MarkerArray()

        # Marker settings for the grid points (free spaces)
        base_marker = Marker()
        base_marker.header.frame_id = "map"
        base_marker.type = Marker.CUBE
        base_marker.action = Marker.ADD
        base_marker.scale.x = 0.1  # Size of each marker
        base_marker.scale.y = 0.1
        base_marker.scale.z = 0.01  # Thin height
        base_marker.color.a = 1.0  # Alpha (transparency)
        base_marker.color.g = 1.0  # Green color for free spaces

        # Generate grid points
        marker_id = 0
        for y in range(map_height):
            for x in range(map_width):
                if grid_array[y, x] == 1:  # Only for free space
                    new_marker = Marker()  # Create a new Marker instance
                    new_marker.header = base_marker.header  # Copy header
                    new_marker.type = base_marker.type  # Copy type
                    new_marker.action = base_marker.action  # Copy action
                    new_marker.scale = base_marker.scale  # Copy scale
                    new_marker.color = base_marker.color  # Copy color

                    new_marker.id = marker_id
                    new_marker.pose.position.x = map_origin_x + x * map_resolution
                    new_marker.pose.position.y = map_origin_y + y * map_resolution
                    new_marker.pose.position.z = 0.0

                    marker_array.markers.append(new_marker)
                    marker_id += 1

        # Publish the marker array
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            self.get_logger().info(f"Published {len(marker_array.markers)} grid markers.")
        else:
            self.get_logger().warn("No markers to publish.")

def main(args=None):
    rclpy.init(args=args)
    node = GridMarkerNode()
    node.get_logger().info("GridMarkerNode spinning")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()





    # def map_callback(self, msg):
    #     self.get_logger().info("Map callback triggered!")
    #     self.get_logger().info(f"Received map with dimensions: {msg.info.width} x {msg.info.height}")

    #     # Extract map information
    #     map_width = msg.info.width
    #     map_height = msg.info.height
    #     map_resolution = msg.info.resolution
    #     map_origin_x = msg.info.origin.position.x
    #     map_origin_y = msg.info.origin.position.y

    #     # Create MarkerArray for grid points
    #     marker_array = MarkerArray()

    #     # Marker settings
    #     base_marker = Marker()
    #     base_marker.header.frame_id = "map"
    #     base_marker.type = Marker.CUBE
    #     base_marker.action = Marker.ADD
    #     base_marker.scale.x = map_resolution
    #     base_marker.scale.y = map_resolution
    #     base_marker.scale.z = 0.05  # Grid height
    #     base_marker.color.a = 1.0  # Alpha (transparency)

    #     # Generate grid points
    #     marker_id = 0
    #     for y in range(map_height):
    #         for x in range(map_width):
    #             index = y * map_width + x
    #             new_marker = Marker()  # Create a new Marker instance
    #             new_marker.header = base_marker.header  # Copy header
    #             new_marker.type = base_marker.type  # Copy type
    #             new_marker.action = base_marker.action  # Copy action
    #             new_marker.scale = base_marker.scale  # Copy scale
    #             new_marker.color.a = base_marker.color.a  # Copy alpha

    #             new_marker.id = marker_id
    #             new_marker.pose.position.x = map_origin_x + x * map_resolution
    #             new_marker.pose.position.y = map_origin_y + y * map_resolution
    #             new_marker.pose.position.z = 0.0

    #             if msg.data[index] == 0:  # Free space
    #                 new_marker.color.g = 1.0  # Green
    #                 new_marker.color.r = 0.0
    #             else:  # Occupied
    #                 new_marker.color.r = 1.0  # Red
    #                 new_marker.color.g = 0.0

    #             marker_array.markers.append(new_marker)
    #             marker_id += 1

    #     # Publish the marker array
    #     if marker_array.markers:
    #         self.marker_pub.publish(marker_array)
    #         self.get_logger().info(f"Published {len(marker_array.markers)} grid markers.")
    #     else:
    #         self.get_logger().warn("No markers to publish.")