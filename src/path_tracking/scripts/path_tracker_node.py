#!/usr/bin/python3
############################################################################
####                        regular import                              ####
############################################################################
import rclpy
from rclpy.node import Node
import csv
import math
import tf_transformations
import sys, traceback
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
############################################################################
####                        import ros msgs                             ####
############################################################################
from std_msgs.msg import String, Int8, Int32MultiArray
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
############################################################################
####                             import tf2                             ####
############################################################################
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Vehicle parameters (m)
LENGTH = 3.0 #vehicle lenght
WIDTH = 2.0 #vehicle width
BACKTOWHEEL = 0.5 #center back vehicle to back of car
WHEEL_LEN = 0.3
WHEEL_WIDTH = 0.2
TREAD = 0.7
WB = 2.0
dt = 0.1

class Vehicle:
	def __init__(self, x, y, yaw, vel=0.0):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.vel = vel
	def update(self, acc, delta):
		"""
		Vehicle motion model, here we are using simple bycicle model
		;param acc: float, acceleration
		;param delta: float, heading control
		"""
		self.x += self.vel*math.cos(self.yaw)*dt
		self.y += self.vel*math.sin(self.yaw)*dt
		self.yaw += self.vel*math.tan(delta)/WB*dt

class PID:
	def __init__(self, kp=0.8, ki=0.1, kd=0.001):
		"""
		Define a PID controller class
		:param kp: float, kp coeff
		:param ki: float, ki coeff
		:param kd: float, kd coeff
		:param ki: float, ki coeff
		"""
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.Pterm = 0.0
		self.Iterm = 0.0
		self.Dterm = 0.0
		self.last_error = 0.0
		
	def control(self, error):
		"""
		PID main function, given an input, this function will output a control unit
		:param error: float, error term
		:return: float, output control
		"""
		self.Pterm = self.kp * error
		self.Iterm += self.ki*error * dt
		self.Dterm += self.kd*error/dt
		self.last_error = error
		output = self.Pterm + self.ki * self.Iterm
		return output

class PurePursuitNode(Node):
	def __init__(self):
		super().__init__('pure_pursuit_node')
		self.declare_parameter('base_frame_id', 'base_link')
		self.declare_parameter('map_frame_id', 'map')
		self.declare_parameter('lookahead_distance', 0.3)
		self.declare_parameter('max_linear_vel', 1.5)
		self.declare_parameter('min_linear_vel', 0.15)
		self.declare_parameter('max_angular_vel', 0.5)
		self.declare_parameter('max_steering', 1.5707) # 1 rad
		self.declare_parameter('wheel_base', 1.2)
		self.declare_parameter('accelerate', 1.0)
		self.declare_parameter('decelerate', 0.5)
		self.declare_parameter('goal_tolerance', 0.05)
		self.declare_parameter('nearest_start', True)
		self.declare_parameter('vehicle_length', 3.0)
		self.declare_parameter('vehicle_width', 2.0)
		self.declare_parameter('wheel_length', 0.3)
		self.declare_parameter('wheel_width', 0.2)
		self.declare_parameter('linear_vel', 0.2)
		self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
		self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
		self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
		self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
		self.min_linear_vel = self.get_parameter('min_linear_vel').get_parameter_value().double_value
		self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
		self.max_steering = self.get_parameter('max_steering').get_parameter_value().double_value
		self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
		self.accelerate = self.get_parameter('accelerate').get_parameter_value().double_value
		self.decelerate = self.get_parameter('decelerate').get_parameter_value().double_value
		self.tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
		self.nearest_start = self.get_parameter('nearest_start').get_parameter_value().bool_value
		self.linear_vel = self.get_parameter('linear_vel').get_parameter_value().double_value
		# parameter
		self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
		# self.path_subscriber = self.create_subscription(Path, 'path_segment', self.path_callback, 10)
		# self.path_publisher = self.create_publisher(Path, 'tracking_path', 10)
		self.curr_publisher = self.create_publisher(Odometry, 'sym_odom', 10)
		self.goal_publisher = self.create_publisher(Odometry, 'target', 10)
		self.cmd_vel_publisher = self.create_publisher(Twist, 'ctrl_vel', 10)
		# init variable
		self.current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec/1e9
		self.curr_odom_time = self.current_time
		self.prev_odom_time = self.current_time
		# initial state
		self.current_pose = None
		self.ctrl_vel = Twist()
		self.curr_pose = Odometry()
		self.goal_pose = Odometry()
		self.reach_point = Odometry()
		# -1.9607907385988916,-0.6240001509797617 turtlebot
		# 0.2, 1.6 f1
		self.curr_pose.pose.pose.position.x = -1.96
		self.curr_pose.pose.pose.position.y = -0.62
		self.curr_heading = 0.0
		self.goal_heading = 0.0
		self.turn_vel = 0.0
		self.curr_time = time.time()
		self.prev_time = time.time()
		self.using_rotation = False
		self.traj_ego_x = []
		self.traj_ego_y = []
		fileName = '/home/nontanan/robinz_ws/src/path_tracking/csv/waypoint.csv'
		self.path_msg = Path()
		self.path_msg.header.frame_id = "map"
		self.waypoints = self.load_waypoints_from_csv(fileName)
		if self.verify_path_data(fileName):
			self.path = self.refrom_path_data(fileName)
		# self.path_time = self.create_timer(0.5, self.publish_path)
		self.plot = True

	def verify_path_data(self, fileName):
		path_reform = []
		df = pd.read_csv(fileName)
		data_x = df['x']
		data_y = df['y']
		data_qx = df['orientation_x']
		data_qx = df['orientation_y']
		data_qy = df['orientation_z']
		data_qw = df['orientation_w']
		if(data_x.shape[0] == data_y.shape[0]):
			return True
		else:
			return False

	def refrom_path_data(self, fileName):
		path_reform = []
		df = pd.read_csv(fileName)
		data_x = df['x']
		data_y = df['y']
		data_qx = df['orientation_x']
		data_qy = df['orientation_y']
		data_qz = df['orientation_z']
		data_qw = df['orientation_w']
		for idx in range(0, len(data_x)-1):
			path_reform.append([data_x[idx], data_y[idx], data_qx[idx], data_qy[idx], data_qz[idx], data_qw[idx]])
		return path_reform

	def publish_path(self):
		self.path_msg.header.stamp = self.get_clock().now().to_msg()
		self.path_msg.poses = []
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
		# Publish the path
		self.path_publisher.publish(self.path_msg)

	def odom_callback(self, msg):
		self.curr_pose = msg
		self.current_pose = msg.pose.pose
	
	def pt_to_pt_distance (self, pt1, pt2):
		pt1 = [pt1.pose.pose.position.x, pt1.pose.pose.position.y]
		pt2 = [pt2.pose.pose.position.x, pt2.pose.pose.position.y]
		distance = np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
		return distance

	def pt_to_pt_distance2(self,pt1,pt2):
		distance = np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
		return distance
	
	# returns -1 if num is negative, 1 otherwise
	def sgn (self, num):
		if num >= 0:
			return 1
		else:
			return -1
	
	def check_better_point(self, path, solutions, currentPos, nearest_index):
		minX = min(path[nearest_index][0], path[nearest_index+1][0])
		minY = min(path[nearest_index][1], path[nearest_index+1][1])
		maxX = max(path[nearest_index][0], path[nearest_index+1][0])
		maxY = max(path[nearest_index][1], path[nearest_index+1][1])
		sol_pt1, sol_pt2 = solutions
		goalPt = path[nearest_index]
		lastFoundIndex = nearest_index
		if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) or ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
			if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
				if self.pt_to_pt_distance2(sol_pt1, path[nearest_index+1]) < self.pt_to_pt_distance2(sol_pt2, path[nearest_index+1]):
					goalPt = sol_pt1
				else:
					goalPt = sol_pt2
			else:
				if (minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY):
					goalPt = sol_pt1
				else:
					goalPt = sol_pt2
			if self.pt_to_pt_distance2(goalPt, path[nearest_index+1]) < self.pt_to_pt_distance2([currentPos.pose.pose.position.x, currentPos.pose.pose.position.y], path[nearest_index+1]):
				lastFoundIndex = nearest_index
			else:
				lastFoundIndex = nearest_index+1
		else:
			goalPt = [path[lastFoundIndex][0], path[lastFoundIndex][1]]
		return lastFoundIndex, goalPt

	def find_nearest_point(self, path, currentPos, lookAheadDis, LFindex):
		lastFoundIndex = LFindex
		goalPt = path[LFindex]  # Default goal point
		for idx in range(LFindex, len(path) - 1):
			x1, y1 = path[idx][0] - currentPos.pose.pose.position.x, path[idx][1] - currentPos.pose.pose.position.y
			x2, y2 = path[idx+1][0] - currentPos.pose.pose.position.x, path[idx+1][1] - currentPos.pose.pose.position.y
			dx, dy = x2 - x1, y2 - y1
			dr = math.sqrt(dx**2 + dy**2)
			D = x1 * y2 - x2 * y1
			discriminant = (lookAheadDis**2) * (dr**2) - D**2
			if discriminant >= 0:
				sol_x1 = (D * dy + self.sgn(dy) * dx * math.sqrt(discriminant)) / dr**2
				sol_x2 = (D * dy - self.sgn(dy) * dx * math.sqrt(discriminant)) / dr**2
				sol_y1 = (-D * dx + abs(dy) * math.sqrt(discriminant)) / dr**2
				sol_y2 = (-D * dx - abs(dy) * math.sqrt(discriminant)) / dr**2
				sol_pt1 = [sol_x1 + currentPos.pose.pose.position.x, sol_y1 + currentPos.pose.pose.position.y]
				sol_pt2 = [sol_x2 + currentPos.pose.pose.position.x, sol_y2 + currentPos.pose.pose.position.y]
				lastFoundIndex, goalPt = self.check_better_point(path, [sol_pt1, sol_pt2], currentPos, idx)
		return lastFoundIndex, goalPt
	
	def path_tracking(self, path, currentPos, currentHeading, lookAheadDis, LFindex):
		goal_return = Odometry()
		nearest_idx, nearest_point = self.find_nearest_point(path, currentPos, lookAheadDis, LFindex)
		Kp = 2.5
		absTargetAngle = math.atan2(nearest_point[1]-currentPos.pose.pose.position.y, nearest_point[0]-currentPos.pose.pose.position.x) *180/np.pi
		if absTargetAngle < 0: absTargetAngle += 360
		turnError = absTargetAngle - currentHeading
		if turnError > 180 or turnError < -180 :
			turnError = -1 * self.sgn(turnError) * (360 - abs(turnError))
		turnVel = Kp*turnError
		goal_return.pose.pose.position.x = nearest_point[0]
		goal_return.pose.pose.position.y = nearest_point[1]
		return goal_return, nearest_idx, turnVel


	def pure_pursuit(self,path, currentPos, currentHeading, lookAheadDis, LFindex) :
		# extract currentX and currentY
		currentX = currentPos.pose.pose.position.x
		currentY = currentPos.pose.pose.position.y
		goal_return = Odometry()
		goalPt = []
		# use for loop to search intersections
		lastFoundIndex = LFindex
		intersectFound = False
		startingIndex = lastFoundIndex
		for i in range (startingIndex, len(path)-1):
			# beginning of line-circle intersection code
			x1 = path[i][0] - currentPos.pose.pose.position.x
			y1 = path[i][1] - currentPos.pose.pose.position.y
			x2 = path[i+1][0] - currentPos.pose.pose.position.x
			y2 = path[i+1][1] - currentPos.pose.pose.position.y
			dx = x2 - x1
			dy = y2 - y1
			dr = math.sqrt (dx**2 + dy**2)
			D = x1*y2 - x2*y1
			discriminant = (lookAheadDis**2) * (dr**2) - D**2
			# print("idx: {}, pt1: {}, pt2: {}".format(i, [x1,y1], [x2,y2]))
			if discriminant >= 0:
				sol_x1 = (D * dy + self.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
				sol_x2 = (D * dy - self.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
				sol_y1 = (- D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2
				sol_y2 = (- D * dx - abs(dy) * np.sqrt(discriminant)) / dr**2

				# sol_pt1 = [sol_x1 + currentX, sol_y1 + currentY]
				# sol_pt2 = [sol_x2 + currentX, sol_y2 + currentY]
				sol_pt1 = [sol_x1 + currentPos.pose.pose.position.x, sol_y1 + currentPos.pose.pose.position.y]
				sol_pt2 = [sol_x2 + currentPos.pose.pose.position.x, sol_y2 + currentPos.pose.pose.position.y]
				# end of line-circle intersection code

				minX = min(path[i][0], path[i+1][0])
				minY = min(path[i][1], path[i+1][1])
				maxX = max(path[i][0], path[i+1][0])
				maxY = max(path[i][1], path[i+1][1])

				# if one or both of the solutions are in range
				if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) or ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
					foundIntersection = True
					# if both solutions are in range, check which one is better
					if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
						# make the decision by compare the distance between the intersections and the next point in path
						if self.pt_to_pt_distance2(sol_pt1, path[i+1]) < self.pt_to_pt_distance2(sol_pt2, path[i+1]):
							goalPt = sol_pt1
						else:
							goalPt = sol_pt2

					# if not both solutions are in range, take the one that's in range
					else:
						# if solution pt1 is in range, set that as goal point
						if (minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY):
							goalPt = sol_pt1
						else:
							goalPt = sol_pt2
			
					# only exit loop if the solution pt found is closer to the next pt in path than the current pos
					if self.pt_to_pt_distance2(goalPt, path[i+1]) < self.pt_to_pt_distance2([currentX, currentY], path[i+1]):
						# update lastFoundIndex and exit
						lastFoundIndex = i
						break
					else:
						# in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
						lastFoundIndex = i+1
				# if no solutions are in range
				else:
					foundIntersection = False
					# no new intersection found, potentially deviated from the path
					# follow path[lastFoundIndex]
					goalPt = [path[lastFoundIndex][0], path[lastFoundIndex][1]]

		# obtained goal point, now compute turn vel
		# initialize proportional controller constant
		Kp = 2.5

		# calculate absTargetAngle with the atan2 function
		# print(goalPt)
		absTargetAngle = math.atan2(goalPt[1]-currentPos.pose.pose.position.y, goalPt[0]-currentPos.pose.pose.position.x) *180/np.pi
		if absTargetAngle < 0: absTargetAngle += 360

		# compute turn error by finding the minimum angle
		turnError = absTargetAngle - currentHeading
		if turnError > 180 or turnError < -180 :
			turnError = -1 * self.sgn(turnError) * (360 - abs(turnError))
	
		# apply proportional controller
		turnVel = Kp*turnError
		# print(dx,dy,discriminant)
		# txt_debug = "dx: {}, dy:{}, d: {}".format(dx,dy,discriminant)
		# print(txt_debug)
		goal_return.pose.pose.position.x = goalPt[0]
		goal_return.pose.pose.position.y = goalPt[1]
		return goal_return, lastFoundIndex, turnVel

	
	def simlulation(self):
		self.curr_heading = 0.0
		self.prev_heading = 330.0
		self.yaw = self.curr_heading*2*np.pi/360.00
		self.yaw_err = 0.0
		self.dt = 0.1
		self.lastFoundIndex = 0
		self.goalPtr = [0.0,0.0]
		self.reach_point.pose.pose.position.x = self.path[len(self.path)-1][0]
		self.reach_point.pose.pose.position.y = self.path[len(self.path)-1][1]

		self.gen_x = []
		self.gen_y = []

		self.curr_pose.header.stamp = self.get_clock().now().to_msg()
		self.curr_pose.header.frame_id = 'odom'
		self.curr_pose.child_frame_id = 'base_link'
		self.goal_pose.header.stamp = self.get_clock().now().to_msg()
		self.goal_pose.header.frame_id = 'odom'
		self.goal_pose.child_frame_id = 'base_link'

		for i in range(0, len(self.path)):
			self.gen_x.append(self.path[i][0])
			self.gen_y.append(self.path[i][1])

		if self.lastFoundIndex <= len(self.path)-2 :
			# plt.figure(figsize=(12, 8))
			plt.figure(figsize=(6, 4))
			# while(self.pt_to_pt_distance(self.curr_pose, self.reach_point) >= self.lookahead_distance) and (self.pt_to_pt_distance(self.curr_pose, self.reach_point) <= 2):
			while(self.pt_to_pt_distance(self.curr_pose, self.reach_point) >= self.lookahead_distance):
				# self.time_stamp = self.get_clock().now()
				# self.curr_time = self.time_stamp.seconds_nanoseconds()[0]+self.time_stamp.seconds_nanoseconds()[1]/1e9
				# self.dt = self.curr_time-self.prev_time
				self.curr_pose.header.stamp = self.get_clock().now().to_msg()
				self.goal_pose.header.stamp = self.get_clock().now().to_msg()
				# self.goal_pose, self.lastFoundIndex, self.turn_vel = self.pure_pursuit(self.path, self.curr_pose, self.curr_heading, self.lookahead_distance, self.lastFoundIndex)
				self.goal_pose, self.lastFoundIndex, self.turn_vel = self.path_tracking(self.path, self.curr_pose, self.curr_heading, self.lookahead_distance, self.lastFoundIndex)
				txt_info = "index: {}, curr_pose: {}, goal_pose: {}. curr_head: {}".format(self.lastFoundIndex, [self.curr_pose.pose.pose.position.x,self.curr_pose.pose.pose.position.y],
																			[self.goal_pose.pose.pose.position.x,self.goal_pose.pose.pose.position.y], self.curr_heading)
				self.dy = self.goal_pose.pose.pose.position.y - self.curr_pose.pose.pose.position.y
				self.dx = self.goal_pose.pose.pose.position.x - self.curr_pose.pose.pose.position.x
				self.yaw_err = math.atan2(self.dy, self.dx)

				self.step_dist = self.linear_vel*self.dt
				self.curr_pose.pose.pose.position.x += self.step_dist*np.cos(self.curr_heading*np.pi/180.0)
				self.curr_pose.pose.pose.position.y += self.step_dist*np.sin(self.curr_heading*np.pi/180.0)
				
				self.curr_heading += self.turn_vel*self.dt
				if self.using_rotation == False:
					self.curr_heading = self.curr_heading%360
					if self.curr_heading <0:
						self.curr_heading += 360
				self.yaw_err = self.yaw_err*2*np.pi/360.00
				self.yaw = self.curr_heading*2.0*np.pi/360.00

				self.traj_ego_x.append(self.curr_pose.pose.pose.position.x)
				self.traj_ego_y.append(self.curr_pose.pose.pose.position.y)
				self.prev_heading = self.curr_heading

				self.curr_quat = quaternion_from_euler(0, 0, self.curr_heading)
				self.goal_quat = quaternion_from_euler(0, 0, self.yaw_err)
				
				self.curr_pose.pose.pose.orientation.x = self.curr_quat[0]
				self.curr_pose.pose.pose.orientation.y = self.curr_quat[1]
				self.curr_pose.pose.pose.orientation.z = self.curr_quat[2]
				self.curr_pose.pose.pose.orientation.w = self.curr_quat[3]
				self.goal_pose.pose.pose.orientation.x = self.curr_quat[0]
				self.goal_pose.pose.pose.orientation.y = self.curr_quat[1]
				self.goal_pose.pose.pose.orientation.z = self.curr_quat[2]
				self.goal_pose.pose.pose.orientation.w = self.curr_quat[3]

				self.curr_publisher.publish(self.curr_pose)
				self.goal_publisher.publish(self.goal_pose)

				# print(txt_info)
				if self.plot:
					self.plot_trajectory()
				# self.prev_time = time.time()
	
	def plot_trajectory(self):
		plt.cla()
		plt.plot(self.gen_x, self.gen_y,"-*",color= "grey", linewidth=1, label="generate course")
		plt.plot(self.traj_ego_x, self.traj_ego_y,"-",color= "blue", linewidth=1, label="trajectory")
		plt.plot(self.curr_pose.pose.pose.position.x, self.curr_pose.pose.pose.position.y,"o" ,color = "red",label="currentPos")
		plt.plot(self.goal_pose.pose.pose.position.x, self.goal_pose.pose.pose.position.y, "og", ms=5, label="target point")
		# plotVehicle(x=currentPos[0], y=currentPos[1], yaw=yaw, steer=yaw_err)
		plt.xlabel("x[m]")
		plt.ylabel("y[m]")
		plt.axis("equal")
		plt.legend()
		plt.grid(True)
		plt.pause(0.1)

	def load_path_from_csv(self, file_path):
		path = Path()
		path.header.frame_id = 'map'

		with open(file_path, 'r') as csv_file:
			csv_reader = csv.reader(csv_file)
			next(csv_reader)  # Skip header row
			for row in csv_reader:
				pose = PoseStamped()
				pose.header.frame_id = 'map'
				pose.pose.position.x = float(row[0])
				pose.pose.position.y = float(row[1])
				pose.pose.position.z = 0.0  # Assuming 2D plane
				pose.pose.orientation.x = float(row[3])
				pose.pose.orientation.y = float(row[4])
				pose.pose.orientation.z = float(row[5])
				pose.pose.orientation.w = float(row[6])
				path.poses.append(pose)
		return path
	
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

def main(args=None):
	rclpy.init(args=args)
	node = PurePursuitNode()
	node.control_loop()
	node.destroy_node()
	rclpy.shutdown()

def path_tracking(args=None):
	rclpy.init(args=args)
	node = PurePursuitNode()
	node.simlulation()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	path_tracking()
