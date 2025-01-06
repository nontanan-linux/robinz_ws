#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class Planner(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.get_logger().info("Planner Node initialized")

        # QoS profile for subscription
        qos_profile = QoSProfile(depth=5)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subscriber to the map
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.get_logger().info("Subscribed to /map")

        # Additional subscribers for odometry and goal pose
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

        # Variable to store the current pose and goal pose
        self.current_pose = PoseStamped()
        self.goal_pose = None
        self.map_msgs = OccupancyGrid()
        self.grid = self.create_grid_array(self.map_msgs)

        # Create a timer to call the debug function
        self.plot_timer = self.create_timer(1.0, self.plot_loop)
        self.monitor = self.create_timer(0.1, self.main)

    def create_grid_array(self, msg):
        """Create a 2D array representing the map."""
        grid_array = np.zeros((msg.info.height, msg.info.width), dtype=int)

        for y in range(msg.info.height):
            for x in range(msg.info.width):
                index = y * msg.info.width + x
                # Convert occupancy values to 0 (occupied) and 1 (free)
                grid_array[y, x] = 0 if msg.data[index] >= 50 else 1  # 50 is the threshold for occupied

        return grid_array

    def plot_grid_array(self, grid_array, map_resolution, map_origin_x, map_origin_y):
        """Plot the grid array using Matplotlib, including the current pose."""
        plt.imshow(grid_array, cmap='gray', origin='lower',
                   extent=(map_origin_x, map_origin_x + grid_array.shape[1] * map_resolution,
                           map_origin_y, map_origin_y + grid_array.shape[0] * map_resolution))
        plt.colorbar(label='Occupancy')
        plt.title('Grid Occupancy Map')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.axis('equal')  # Keep the aspect ratio square

        # If current pose is set, plot it
        if self.current_pose is not None:
            current_x = self.current_pose.pose.pose.position.x
            current_y = self.current_pose.pose.pose.position.y
            plt.plot(current_x, current_y, marker='o', color='green', markersize=10, label='Current Pose')

        # If goal pose is set, plot it
        if self.goal_pose is not None:
            goal_x = self.goal_pose.pose.pose.position.x
            goal_y = self.goal_pose.pose.pose.position.y
            plt.plot(goal_x, goal_y, marker='x', color='red', markersize=10, label='Goal Pose')

        plt.legend()
        plt.show()

    def map_callback(self, msg):
        """Callback for the map subscriber."""
        self.map_msgs = msg
        self.grid = self.create_grid_array(msg)
        # self.get_logger().info(f"Grid array created with shape: {grid_array.shape}")
        # self.plot_grid_array(grid_array, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)

    def odom_callback(self, msg):
        """Callback for odometry data."""
        self.current_pose = msg  # Store the current pose from the odometry message
        # self.get_logger().info(f"Received Odometry: (X: {msg.pose.pose.position.x}, Y: {msg.pose.pose.position.y})")  # Debug print

    def goal_callback(self, msg):
        """Callback for goal pose data."""
        self.goal_pose = msg  # Store the goal pose
        # self.get_logger().info(f"Received Goal Pose: (X: {msg.pose.position.x}, Y: {msg.pose.position.y})")  # Debug print

    def plot_loop(self):
        # print(self.grid)
        # print(self.map_msgs.info.resolution, self.map_msgs.info.origin.position.x,self.map_msgs.info.origin.position.y,)
        self.plot_grid_array(self.grid, self.map_msgs.info.resolution, self.map_msgs.info.origin.position.x, self.map_msgs.info.origin.position.y)
    
    def main(self):
        print("resolution: {}, origin: {}, height: {}, width: {}".format(
            self.map_msgs.info.resolution, self.map_msgs.info.origin, self.map_msgs.info.height, self.map_msgs.info.width
        ))
        # curr_pose = [self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y]
        # if self.goal_pose == None:
        #     goal_pose = [0.0, 0.0]
        # else:
        #     goal_pose = [self.goal_pose.pose.pose.position.x, self.goal_pose.pose.pose.position.y]
        # print("curr_pose: {}, goal_pose: {}".format(curr_pose, goal_pose))

def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    node.get_logger().info("Planner Node spinning")
    print(node.map_msgs.info.resolution, node.map_msgs.info.origin.position.x,node.map_msgs.info.origin.position.y,)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()





# class AStarPlanner:
#     def __init__(self, grid, width, height):
#         self.grid = grid
#         self.width = width
#         self.height = height

#     def is_valid(self, x, y):
#         # Check if a grid cell is valid (within bounds and not an obstacle)
#         return 0 <= x < self.width and 0 <= y < self.height and self.grid[y * self.width + x] == 0

#     def heuristic(self, start, goal):
#         # Use Manhattan distance as the heuristic
#         return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

#     def plan(self, start, goal):
#         # A* algorithm implementation
#         open_list = []
#         heapq.heappush(open_list, (0 + self.heuristic(start, goal), 0, start))
#         came_from = {}
#         g_score = {start: 0}

#         while open_list:
#             _, current_g, current = heapq.heappop(open_list)

#             if current == goal:
#                 # Reconstruct path from start to goal
#                 path = []
#                 while current in came_from:
#                     path.append(current)
#                     current = came_from[current]
#                 path.append(start)
#                 path.reverse()  # Reverse to get the correct order
#                 return path

#             for neighbor in self.get_neighbors(current):
#                 tentative_g_score = current_g + 1  # Assuming uniform cost for each move
#                 if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                     g_score[neighbor] = tentative_g_score
#                     priority = tentative_g_score + self.heuristic(neighbor, goal)
#                     heapq.heappush(open_list, (priority, tentative_g_score, neighbor))
#                     came_from[neighbor] = current

#         return None  # No path found

#     def get_neighbors(self, node):
#         x, y = node
#         # 4-connected grid: up, down, left, right
#         neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
#         # Filter out invalid neighbors
#         valid_neighbors = [(nx, ny) for nx, ny in neighbors if self.is_valid(nx, ny)]
#         return valid_neighbors


# class GridPathPlanner(Node):
#     def __init__(self):
#         super().__init__('grid_path_planner')
#         self.get_logger().info("grid_path_planner initialized")
#         # QoS profile for subscription
#         qos_profile = QoSProfile(depth=5)
#         qos_profile.reliability = ReliabilityPolicy.RELIABLE
#         qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
#         self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
#         self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
#         self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

#         # Variables to store the map data, robot's current pose, and goal pose
#         self.grid = None
#         self.width = 0
#         self.height = 0
#         self.resolution = 0.005
#         self.current_pose = None  # To store the robot's current position (x, y)
#         self.goal_pose = None  # To store the goal position (x, y)
    
#     def create_grid_array(self, msg):
#         """
#         Create a 2D array representing the map.
#         Black (occupied) is 0, and white (free) is 1.
#         """
#         grid_array = np.zeros((msg.info.height, msg.info.width), dtype=int)

#         for y in range(msg.info.height):
#             for x in range(msg.info.width):
#                 index = y * msg.info.width + x
#                 # Convert occupancy values to 0 (occupied) and 1 (free)
#                 if msg.data[index] >= 50:  # Occupied if value is >= 50
#                     grid_array[y, x] = 0  # Black (occupied)
#                 else:
#                     grid_array[y, x] = 1  # White (free)
#         return grid_array

#     def map_callback(self, msg):
#         # Callback function to handle the map data (OccupancyGrid)
#         self.grid = self.create_grid_array(msg)
#         self.width = msg.info.width
#         self.height = msg.info.height
#         self.resolution = msg.info.resolution
#         self.get_logger().info(f'Map received: {self.width}x{self.height}, resolution: {self.resolution}')

#     def odom_callback(self, msg):
#         # Callback function to handle the robot's current pose
#         self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
#         self.get_logger().info(f'Current pose: {self.current_pose}')

#         # If the goal pose is already set, trigger path planning
#         if self.goal_pose:
#             self.plan_and_publish_path(self.current_pose, self.goal_pose)

#     def goal_callback(self, msg):
#         # Callback function to handle the goal pose
#         self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
#         self.get_logger().info(f'Goal pose received: {self.goal_pose}')

#         # If the current pose is already set, trigger path planning
#         if self.current_pose:
#             self.plan_and_publish_path(self.current_pose, self.goal_pose)

#     def plan_and_publish_path(self, start_pose, goal_pose):
#         if self.grid is None or self.resolution == 0.0:
#             self.get_logger().warn('Map not received or resolution is zero, cannot plan the path')
#             return
        
#         # Convert the world coordinates (start_pose and goal_pose) to grid coordinates
#         start_grid = self.world_to_grid(start_pose)
#         goal_grid = self.world_to_grid(goal_pose)

#         self.get_logger().info(f'Planning path from {start_grid} to {goal_grid}')

#         # Plan the path using the A* algorithm
#         planner = AStarPlanner(self.grid, self.width, self.height)
#         grid_path = planner.plan(start_grid, goal_grid)

#         if grid_path:
#             self.get_logger().info(f'Path found: {grid_path}')
#             self.publish_path(grid_path)
#         else:
#             self.get_logger().warn('No path found')


#     def world_to_grid(self, pose):
#         # Convert world coordinates to grid coordinates
#         x_grid = int(pose[0] / self.resolution)
#         y_grid = int(pose[1] / self.resolution)
#         return (x_grid, y_grid)

#     def publish_path(self, grid_path):
#         # Convert the grid path to a nav_msgs/Path and publish it
#         path_msg = Path()
#         path_msg.header.frame_id = 'map'
#         path_msg.header.stamp = self.get_clock().now().to_msg()

#         for (x, y) in grid_path:
#             pose = PoseStamped()
#             pose.header.frame_id = 'map'
#             pose.header.stamp = self.get_clock().now().to_msg()

#             # Convert grid coordinates to world coordinates using resolution
#             pose.pose.position.x = x * self.resolution
#             pose.pose.position.y = y * self.resolution
#             pose.pose.orientation.w = 1.0  # Orientation is not considered here

#             path_msg.poses.append(pose)

#         # Publish the path
#         self.path_publisher.publish(path_msg)
#         self.get_logger().info('Path published!')


# def main(args=None):
#     rclpy.init(args=args)
#     node = GridPathPlanner()
#     rclpy.spin(node)
#     rclpy.shutdown()