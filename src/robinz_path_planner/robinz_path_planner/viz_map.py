import yaml
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import csv

class MapVisualizer:
    def __init__(self, dir, yaml_path):
        self.map_data = self.load_yaml(dir+yaml_path)
        print("map : ", self.map_data['image'])
        self.map_img = self.load_pgm_image(dir+self.map_data['image'])
        self.resolution = self.map_data['resolution']
        self.origin = self.map_data['origin']
        self.waypoints = []  # Array to store waypoints
        self.idx = 0
        self.csv_file_path = dir + 'waypoint.csv'
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header of the CSV file
        self.csv_writer.writerow(['point no.', 'x (meters)', 'y (meters)'])
        
    def load_yaml(self, yaml_path):
        with open(yaml_path, 'r') as file:
            map_config = yaml.safe_load(file)
        return map_config

    def load_pgm_image(self, image_path):
        img = Image.open(image_path)
        img = np.array(img)
        return img

    def transform_point(self, point, translation, scale):
        # Apply scaling and translation to the point
        return [(point[0] * scale[0]) + translation[0], (point[1] * scale[1]) + translation[1]]

    def on_click(self, event):
        if event.inaxes is not None:  # Ensure the click is within the axes
            x = event.xdata
            y = event.ydata
            
            # Mark the clicked point
            plt.scatter(x, y, color='red')
            self.waypoints.append((x, y))  # Add point to the waypoint array
            self.csv_writer.writerow([self.idx, x, y])
            self.idx += 1
            
            # Print the current waypoints
            # print("Current Waypoints:", self.waypoints)

            # Connect to previous point with a line if there is one
            if len(self.waypoints) > 1:
                prev_x, prev_y = self.waypoints[-2]
                plt.plot([prev_x, x], [prev_y, y], color='blue', linewidth=2)  # Draw line

            plt.draw()  # Redraw the figure to show the point and line

    def visualize(self):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map_img, cmap='gray')
        plt.title(f"Map Visualization\nResolution: {self.resolution} m/pixel, Adjusted Origin: {self.origin}")
        plt.gca().invert_yaxis()  # Invert y-axis to match map orientation

        plt.gcf().canvas.mpl_connect('button_press_event', self.on_click)
        plt.show()

if __name__ == "__main__":
    dir = '/home/nontanan/robinz_ws/src/robinz_vehicle_launch/maps/'
    yaml_file = 'test_map_panal.yaml'  # Replace with your YAML file path
    
    # Create an instance of the MapVisualizer class
    visualizer = MapVisualizer(dir, yaml_file)

    # Given points
    point1_original = np.array([2.2, 50.8])
    point1_new = np.array([2.77, 0.0158])
    point2_original = np.array([104.1, 50.3])
    point2_new = np.array([2.28, 0.00419])

    # Calculate the translation vector
    translation = point1_new - point1_original

    # Translate the original points
    point1_translated = point1_original + translation
    point2_translated = point2_original + translation

    # Calculate scaling factors based on the translation of point2
    scale_x = (point2_new[0] - point1_new[0]) / (point2_translated[0] - point1_translated[0])
    scale_y = (point2_new[1] - point1_new[1]) / (point2_translated[1] - point1_translated[1])
    scale = np.array([scale_x, scale_y])

    # Adjust the origin based on the translation
    adjusted_origin = visualizer.transform_point(visualizer.origin, translation, [1, 1])  # No scaling applied to the origin

    # Visualize the map with the adjusted origin
    visualizer.visualize()




# import yaml
# import matplotlib.pyplot as plt
# import numpy as np
# from PIL import Image

# def load_yaml(yaml_path):
#     with open(yaml_path, 'r') as file:
#         map_config = yaml.safe_load(file)
#     return map_config

# def load_pgm_image(image_path):
#     img = Image.open(image_path)
#     img = np.array(img)
#     return img

# def transform_point(point, translation, scale):
#     # Apply scaling and translation to the point
#     return [(point[0] * scale[0]) + translation[0], (point[1] * scale[1]) + translation[1]]

# def visualize_map(map_img, resolution, origin):
#     plt.figure(figsize=(10, 10))
#     plt.imshow(map_img, cmap='gray')
#     plt.title(f"Map Visualization\nResolution: {resolution} m/pixel, Adjusted Origin: {origin}")
#     plt.gca().invert_yaxis()  # Invert y-axis to match map orientation
#     plt.show()

# if __name__ == "__main__":
#     dir = '/home/nontanan/robinz_ws/src/robinz_vehicle_launch/maps/'
#     yaml_file = dir + 'test_map_panal.yaml'  # Replace with your YAML file path
#     map_data = load_yaml(yaml_file)
    
#     image_file = map_data['image']
#     resolution = map_data['resolution']
#     origin = map_data['origin']

#     # Load the PGM image
#     map_img = load_pgm_image(dir + image_file)

#     # Given points
#     point1_original = np.array([2.2, 50.8])
#     point1_new = np.array([2.77, 0.0158])
#     point2_original = np.array([104.1, 50.3])
#     point2_new = np.array([2.28, 0.00419])

#     # Calculate the translation vector
#     translation = point1_new - point1_original

#     # Translate the original points
#     point1_translated = point1_original + translation
#     point2_translated = point2_original + translation

#     # Calculate scaling factors based on the translation of point2
#     scale_x = (point2_new[0] - point1_new[0]) / (point2_translated[0] - point1_translated[0])
#     scale_y = (point2_new[1] - point1_new[1]) / (point2_translated[1] - point1_translated[1])
#     scale = np.array([scale_x, scale_y])

#     # Adjust the origin based on the translation
#     adjusted_origin = transform_point(origin, translation, [1, 1])  # No scaling applied to the origin

#     # Visualize the map with the adjusted origin
#     visualize_map(map_img, resolution, adjusted_origin)

# for this code i create mark point by click