#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from queue import PriorityQueue
import math

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_grid')

        # Robot's current pose
        self.current_pose = None

        # The occupancy grid map
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info("Grid-based Path Planner Node has been started.")

    def map_callback(self, map_msg: OccupancyGrid):
        """Callback to store the map data."""
        self.map_data = map_msg.data
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.map_resolution = map_msg.info.resolution
        self.map_origin_x = map_msg.info.origin.position.x
        self.map_origin_y = map_msg.info.origin.position.y
        self.get_logger().info("Map data received.")

    def odom_callback(self, odom_msg: Odometry):
        """Callback to store the robot's current pose."""
        self.current_pose = odom_msg.pose.pose
        self.get_logger().info(f"Robot pose updated: x={self.current_pose.position.x}, y={self.current_pose.position.y}")

    def goal_callback(self, goal_msg: PoseStamped):
        """Callback when a new goal is received."""
        if self.current_pose is None or self.map_data is None:
            self.get_logger().warn("Current pose or map data not available yet.")
            return

        self.get_logger().info(f"Received goal: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")

        # Plan the path using A*
        path = self.plan_path(self.current_pose, goal_msg.pose)

        # Publish the path
        self.path_pub.publish(path)

    def plan_path(self, start: Pose, goal: Pose):
        """Plan the path using A* algorithm from the start to the goal."""
        start_grid = self.world_to_grid(start.position.x, start.position.y)
        goal_grid = self.world_to_grid(goal.position.x, goal.position.y)

        self.get_logger().info(f"Planning path from {start_grid} to {goal_grid}")

        open_list = PriorityQueue()
        open_list.put((0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}

        came_from[start_grid] = None

        while not open_list.empty():
            current_cost, current = open_list.get()

            if current == goal_grid:
                break

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.get_cost(neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    priority = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    open_list.put((priority, neighbor))
                    came_from[neighbor] = current

        path = self.reconstruct_path(came_from, start_grid, goal_grid)
        return self.grid_to_path(path)

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        return (grid_x, grid_y)

    def get_neighbors(self, grid):
        """Get valid neighboring cells in the occupancy grid."""
        neighbors = []
        x, y = grid
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor_x, neighbor_y = x + dx, y + dy
            if 0 <= neighbor_x < self.map_width and 0 <= neighbor_y < self.map_height:
                neighbors.append((neighbor_x, neighbor_y))
        return neighbors

    def get_cost(self, grid):
        """Get the cost of moving to a grid cell (e.g., 1 for free space, high for obstacles)."""
        x, y = grid
        index = y * self.map_width + x
        cost = self.map_data[index]
        return cost if cost >= 0 else 1  # Treat unknown cells as free space

    def heuristic(self, a, b):
        """Heuristic function for A* (Euclidean distance)."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def reconstruct_path(self, came_from, start, goal):
        """Reconstruct the path from start to goal."""
        current = goal
        path = []
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def grid_to_path(self, path_grid):
        """Convert grid-based path to PoseStamped Path for ROS."""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for grid_x, grid_y in path_grid:
            x = grid_x * self.map_resolution + self.map_origin_x
            y = grid_y * self.map_resolution + self.map_origin_y
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down path planner.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry, Path
# from geometry_msgs.msg import Pose, PoseStamped
# import math


# class PathPlannerNode(Node):
#     def __init__(self):
#         super().__init__('path_planner_node')

#         # Variables to store the robot's current pose
#         self.current_pose = None

#         # Subscribers
#         self.goal_sub = self.create_subscription(
#             PoseStamped,
#             'goal_pose',
#             self.goal_callback,
#             10
#         )

#         self.odom_sub = self.create_subscription(
#             Odometry,
#             'odom',
#             self.odom_callback,
#             10
#         )

#         # Publishers
#         self.path_pub = self.create_publisher(Path, 'planned_path', 10)

#         self.get_logger().info("Path Planner Node has been started.")

#     def odom_callback(self, odom_msg: Odometry):
#         """Callback function to update the current pose of the robot."""
#         self.current_pose = odom_msg.pose.pose
#         self.get_logger().info(f'Updated current pose: x={self.current_pose.position.x}, y={self.current_pose.position.y}')

#     def goal_callback(self, goal_msg: PoseStamped):
#         """Callback function when a new goal is received."""
#         if self.current_pose is None:
#             self.get_logger().warn('Current pose not available yet. Please wait.')
#             return

#         self.get_logger().info(f'Received goal: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}')

#         # Plan the path
#         path = self.plan_path(self.current_pose, goal_msg.pose)

#         # Publish the planned path
#         self.path_pub.publish(path)

#     def plan_path(self, start: Pose, goal: Pose):
#         """Plan a path from the start pose to the goal pose."""
#         path = Path()
#         path.header.frame_id = 'map'

#         # Number of waypoints
#         num_waypoints = 10

#         for i in range(num_waypoints + 1):
#             t = i / num_waypoints

#             # Interpolate between start and goal positions
#             x = (1 - t) * start.position.x + t * goal.position.x
#             y = (1 - t) * start.position.y + t * goal.position.y

#             # Create a new PoseStamped for each waypoint
#             waypoint = PoseStamped()
#             waypoint.pose.position.x = x
#             waypoint.pose.position.y = y
#             waypoint.pose.orientation.w = 1.0  # Simplified for 2D planning

#             # Add to path
#             path.poses.append(waypoint)

#         self.get_logger().info(f'Planned path with {len(path.poses)} waypoints.')
#         return path


# def main(args=None):
#     rclpy.init(args=args)
#     node = PathPlannerNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Shutting down path planner...')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Odometry, Path
# import numpy as np

# class StateLatticePlanner(Node):
#     def __init__(self):
#         super().__init__('state_lattice_planner')

#         # Publisher for the planned path
#         self.path_publisher = self.create_publisher(Path, 'planned_path', 10)

#         # Subscriber for goal position
#         self.goal_subscriber = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,10)

#         # Subscriber for odometry data
#         self.odom_subscriber = self.create_subscription(Odometry,'odom',self.odom_callback,10)

#         self.current_position = (0.0, 0.0)  # Default current position
#         self.get_logger().info("State Lattice Planner Node Initialized")

#     def odom_callback(self, msg: Odometry):
#         # Update the current position from odometry data
#         self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

#     def generate_lattice(self, start, goal):
#         lattice_points = []
#         num_points = 10  # Number of points in the lattice
#         for i in range(num_points + 1):
#             # Linear interpolation between start and goal
#             x = start[0] + (goal[0] - start[0]) * (i / num_points)
#             y = start[1] + (goal[1] - start[1]) * (i / num_points)
#             lattice_points.append((x, y))
#         return lattice_points

#     def cost_function(self, path):
#         # Simple cost function based on distance
#         return sum(np.linalg.norm(np.array(path[i]) - np.array(path[i + 1])) for i in range(len(path) - 1))

#     def goal_callback(self, msg: PoseStamped):
#         # Use current position from odometry
#         goal_position = (msg.pose.position.x, msg.pose.position.y)

#         # Generate the lattice
#         lattice_points = self.generate_lattice(self.current_position, goal_position)

#         # Evaluate the cost of the generated path
#         cost = self.cost_function(lattice_points)
#         self.get_logger().info(f'Cost of path: {cost}')

#         # Publish the planned path
#         path_msg = Path()
#         path_msg.header.frame_id = "map"  # Set the frame ID
#         path_msg.header.stamp = self.get_clock().now().to_msg()

#         for point in lattice_points:
#             pose = PoseStamped()
#             pose.header.frame_id = "map"  # Ensure this matches your frame of reference
#             pose.header.stamp = path_msg.header.stamp
#             pose.pose.position.x = point[0]
#             pose.pose.position.y = point[1]
#             pose.pose.position.z = 0.0  # Set Z to 0 if you're working in 2D
#             pose.pose.orientation.w = 1.0  # Default orientation
#             path_msg.poses.append(pose)

#         # Publish the path message
#         self.path_publisher.publish(path_msg)
#         self.get_logger().info(f'Published path from {self.current_position} to {goal_position}')

# def main(args=None):
#     rclpy.init(args=args)
#     state_lattice_planner = StateLatticePlanner()
#     rclpy.spin(state_lattice_planner)

#     state_lattice_planner.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
