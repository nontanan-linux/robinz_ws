#!/usr/bin/python3
import yaml
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import csv
import heapq
import rclpy
from rclpy.node import Node as ROSNode  # Avoid conflict by aliasing
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class GridNode:
    """Represents a node in the grid."""
    def __init__(self, position, g=0, h=0, parent=None):
        self.position = position
        self.g = g  # Cost from start to this node
        self.h = h  # Heuristic cost to goal
        self.f = g + h  # Total cost
        self.parent = parent  # Reference to the parent node

    def __lt__(self, other):
        return self.f < other.f


class AStarPathfinder:
    """Implements the A* algorithm to find a path on a grid."""
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, a, b):
        """Calculates the Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def find_path(self, start, goal):
        """Finds the shortest path from start to goal using A* algorithm."""
        open_list = []
        closed_list = set()
        
        start = tuple(start)
        goal = tuple(goal)

        start_node = GridNode(start, 0, self.heuristic(start, goal))
        heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)
            closed_list.add(current_node.position)

            if current_node.position == goal:
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]  # Return reversed path

            neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
            for new_position in neighbors:
                node_position = (current_node.position[0] + new_position[0],
                                 current_node.position[1] + new_position[1])
                
                if (0 <= node_position[0] < self.rows and
                    0 <= node_position[1] < self.cols):
                    
                    if self.grid[node_position[0]][node_position[1]] == 0:  # 0 means obstacle
                        continue

                    if node_position in closed_list:
                        continue

                    g_cost = current_node.g + 1
                    h_cost = self.heuristic(node_position, goal)
                    neighbor_node = GridNode(node_position, g_cost, h_cost, current_node)

                    if all(neighbor_node.f < open_node.f for open_node in open_list if open_node.position == node_position):
                        heapq.heappush(open_list, neighbor_node)

        return None  # Path not found


class MapVisualizer:
    def __init__(self, dir, yaml_path):
        self.map_data = self.load_yaml(dir + yaml_path)
        print("Map: ", self.map_data['image'])
        self.map_img = self.load_pgm_image(dir + self.map_data['image'])
        self.resolution = self.map_data['resolution']
        self.origin = self.map_data['origin']
        self.waypoints = []  # Array to store waypoints
        self.idx = 0
        self.csv_file_path = dir + 'waypoint.csv'
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        print(f"resolution: {self.resolution}, origin: {self.origin}")
        
        self.curr_pose = [-2.433, 0.004]
        self.goal_pose = [0.803, 1.943]
        
        self.height, self.width = self.map_img.shape
        self.grid_array = self.create_grid_array(self.map_img, self.height, self.width)
        self.pathfinder = AStarPathfinder(self.grid_array)
        self.curr_grid = self.map_position_to_grid(self.curr_pose)
        self.goal_grid = self.map_position_to_grid(self.goal_pose)

        print(f"Grid start: {self.curr_grid}, Grid goal: {self.goal_grid}")
        
        self.csv_writer.writerow(['point no.', 'x (meters)', 'y (meters)'])
        self.path = self.pathfinder.find_path(self.curr_grid, self.goal_grid)

        if self.path:
            # print("path generated.")
            print(f"Path: {self.path}")
        else:
            print("No path found.")

    def load_yaml(self, yaml_path):
        with open(yaml_path, 'r') as file:
            map_config = yaml.safe_load(file)
        return map_config

    def load_pgm_image(self, image_path):
        img = Image.open(image_path)
        img = np.array(img)
        return img

    def create_grid_array(self, img, height, width):
        grid_array = np.zeros((height, width), dtype=int)
        for y in range(height):
            for x in range(width):
                pixel_value = img[y, x]
                if pixel_value > 200:  # Free space
                    grid_array[y, x] = 1
                else:  # Obstacle
                    grid_array[y, x] = 0
        return grid_array

    def map_position_to_grid(self, position):
        x_real, y_real = position
        x_origin = self.origin[0]
        y_origin = self.origin[1]

        x_grid = int((x_real - x_origin) / self.resolution)
        y_grid = int((y_real - y_origin) / self.resolution)

        if 0 <= x_grid < self.width and 0 <= y_grid < self.height:
            return (y_grid, x_grid)  # Return in row-column format
        else:
            print("Position out of bounds.")
            return None
    def visualize_grid(self):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid_array, cmap='gray', origin='lower',
                extent=(self.origin[0], self.origin[0] + self.width * self.resolution,
                        self.origin[1], self.origin[1] + self.height * self.resolution))

        # Convert path coordinates from grid to real-world coordinates
        if self.path:
            path_real_x = [(point[1] * self.resolution + self.origin[0]) for point in self.path]
            path_real_y = [(point[0] * self.resolution + self.origin[1]) for point in self.path]

            # Plot the path
            plt.plot(path_real_x, path_real_y, color='blue', label='Planned Path', linewidth=2)

            # Plot the start (first point) in green
            plt.scatter(path_real_x[0], path_real_y[0], color='green', label='Start', s=100)

            # Plot the goal (last point) in red
            plt.scatter(path_real_x[-1], path_real_y[-1], color='red', label='Goal', s=100)

        plt.title(f"Grid Visualization with Path Planning\nResolution: {self.resolution} m/pixel")
        plt.gca().invert_yaxis()  # Invert y-axis to match map orientation
        plt.colorbar(label='Occupancy')
        plt.legend()
        plt.show()


    # def visualize_grid(self):
    #     plt.figure(figsize=(10, 10))
    #     plt.imshow(self.grid_array, cmap='gray', origin='lower',
    #             extent=(self.origin[0], self.origin[0] + self.width * self.resolution,
    #                     self.origin[1], self.origin[1] + self.height * self.resolution))

    #     # Plot the path if it exists
    #     if self.path:
    #         # path_x_pixels = [(point[1] * self.resolution + self.origin[0]) for point in self.path]
    #         # path_y_pixels = [(point[0] * self.resolution + self.origin[1]) for point in self.path]
    #         path_x_pixels = [(point[1]*self.resolution + self.origin[0]) for point in self.path]
    #         path_y_pixels = [(point[0]*self.resolution + self.origin[1]) for point in self.path]

    #         # Plot the path
    #         plt.plot(path_x_pixels, path_y_pixels, color='blue', label='Planned Path', linewidth=2)

    #         # Plot the start (first point) in green
    #         plt.scatter(path_x_pixels[0], path_y_pixels[0], color='green', label='Start', s=100)

    #         # Plot the goal (last point) in red
    #         plt.scatter(path_x_pixels[-1], path_y_pixels[-1], color='red', label='Goal', s=100)

    #     plt.title(f"Grid Visualization with Path Planning\nResolution: {self.resolution} m/pixel")
    #     plt.gca().invert_yaxis()  # Invert y-axis to match map orientation
    #     plt.colorbar(label='Occupancy')
    #     plt.legend()
    #     plt.show()




def main():
    dir = '/home/nontanan/robinz_ws/src/robinz_vehicle_launch/maps/'
    yaml_file = 'test_map_panal.yaml'
    visualizer = MapVisualizer(dir, yaml_file)
    visualizer.visualize_grid()

if __name__ == "__main__":
    main()
