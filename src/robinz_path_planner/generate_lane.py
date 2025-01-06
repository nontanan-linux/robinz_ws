import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math

class GenerateLane:
    def __init__(self):
        self.road_width = 0.5 #m
        self.file_path = '/home/nontanan/robinz_ws/src/robinz_vehicle_launch/csv/node.csv'
        self.df = pd.read_csv(self.file_path)
        self.rx = self.df['x'].to_list()
        self.ry = self.df['y'].to_list()
        self.lane_width = 0.20
        self.ori_path = []
        for i in range(0, len(self.rx)):
              self.ori_path.append([self.rx[i], self.ry[i]])
        self.smh_path = self.add_more_points2(self.ori_path,0.1)
        self.smh_path = self.autoSmooth(self.smh_path, 70)
        gen_x = []
        gen_y = []
        for i in range (0,len(self.smh_path)):
            gen_x.append(self.smh_path[i][0])
            gen_y.append(self.smh_path[i][1])
        self.smh_lane = self.generate_lane(gen_x, gen_y)

    def generate_lane(self, rx, ry):
        left_lane = []
        right_lane = []
        for i in range(len(rx) - 1):
            # Get the direction vector between two consecutive points
            dx = rx[i+1] - rx[i]
            dy = ry[i+1] - ry[i]
            # Normalize the direction vector
            length = np.sqrt(dx**2 + dy**2)
            unit_dx = dx / length
            unit_dy = dy / length
            # Perpendicular vector (rotate by 90 degrees)
            perp_dx = -unit_dy
            perp_dy = unit_dx
            # Offset points for left and right lanes
            left_x = rx[i] + perp_dx * self.lane_width / 2
            left_y = ry[i] + perp_dy * self.lane_width / 2
            right_x = rx[i] - perp_dx * self.lane_width / 2
            right_y = ry[i] - perp_dy * self.lane_width / 2
            left_lane.append((left_x, left_y))
            right_lane.append((right_x, right_y))
        # Add the last points
        left_lane.append((rx[-1] + perp_dx * self.lane_width / 2, ry[-1] + perp_dy * self.lane_width / 2))
        right_lane.append((rx[-1] - perp_dx * self.lane_width / 2, ry[-1] - perp_dy * self.lane_width / 2))
        return left_lane, right_lane
    
    # specify the distance between adjacent points
    def add_more_points2(self, path,segment_length):
        new_path = []
        for i in range (0,len(path)-1):
            distance = np.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
            num_of_points = int(round(distance/segment_length))
            if num_of_points == 0:
                new_path.append(path[i])
            else:
                segment_x = (path[i+1][0] - path[i][0]) / num_of_points
                segment_y = (path[i+1][1] - path[i][1]) / num_of_points
                for j in range (0,num_of_points):
                    new_point = [(path[i][0] + j*segment_x),(path[i][1] + j*segment_y)]
                    new_path.append(new_point)
        new_path.append(path[-1])
        return new_path
    
    def autoSmooth(self, path, maxAngle) :
        currentMax = 0
        param = 0.01
        new_path = path
        firstLoop = True
        counter = 0
        while (currentMax >= maxAngle or firstLoop == True) : # and counter <= 15 :
            param += 0.01
            firstLoop = False
            # counter += 1
            # print('this is the {} iteration'.format(counter))
            new_path = self.smoothing (path, 0.1, param, 0.1)
            currentMax = 0

            for i in range (1, len(new_path)-2) :
                angle1 = math.atan2(new_path[i][1] - new_path[i-1][1], new_path[i][0] - new_path[i-1][0]) *180/np.pi
                if angle1 < 0 : angle1 += 360
                angle2 = math.atan2(new_path[i+1][1] - new_path[i][1], new_path[i+1][0] - new_path[i][0]) *180/np.pi
                if angle2 < 0 : angle2 += 360
                if abs(self.findMinAngle(angle2, angle1)) > currentMax :
                    currentMax = abs(self.findMinAngle(angle2, angle1))
        return new_path

    def smoothing(self, path,weight_data,weight_smooth,tolerance):
        smoothed_path = path.copy()
        change = tolerance
        while change >= tolerance :
            change = 0.0
            for i in range (1,len(path)-1):
                for j in range (0,len(path[i])):
                    aux = smoothed_path[i][j]
                    smoothed_path[i][j] += weight_data * (path[i][j] - smoothed_path[i][j]) + weight_smooth * (smoothed_path[i-1][j] + smoothed_path[i+1][j] - (2.0 * smoothed_path[i][j]))
                    change += np.abs(aux - smoothed_path[i][j])
        return smoothed_path

    def findMinAngle(self, absTargetAngle, currentHeading) :
        minAngle = absTargetAngle - currentHeading
        if minAngle > 180 or minAngle < -180 :
            minAngle = -1 * self.sgn(minAngle) * (360 - abs(minAngle))
        return minAngle

    def sgn (self, num):
        if num >= 0:
            return 1
        else:
            return -1
    
def main():
    gl = GenerateLane()
    # left_lane, right_lane = gl.generate_lane()
    left_lane = gl.smh_lane[0]
    right_lane = gl.smh_lane[1]

    # Separate x and y for plotting
    left_x, left_y = zip(*left_lane)
    right_x, right_y = zip(*right_lane)

    gen_x = []
    gen_y = []
    for i in range (0,len(gl.smh_path)):
        gen_x.append(gl.smh_path[i][0])
        gen_y.append(gl.smh_path[i][1])

    # Plot the center line and the lanes
    plt.plot(gen_x, gen_y, 'k-*', label="Center Line")
    plt.plot(left_x, left_y, 'r-', label="Left Lane")
    plt.plot(right_x, right_y, 'b-', label="Right Lane")
    plt.axis('equal')
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Lane Generation from Line')
    plt.show()

if __name__ == '__main__':
    main()

# some path of lift lane is not work it Overlapping, not like the edge of the lane