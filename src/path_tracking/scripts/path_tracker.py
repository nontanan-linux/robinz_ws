#!/usr/bin/python3
############################################################################
####                        regular import                              ####
############################################################################
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import csv
import math
import sys, traceback
import numpy as np
import time
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
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class PID:
	def __init__(self, kp=0.8, ki=0.1, kd=0.001):
		self.kp = kp		#:param kp: float, kp coeff
		self.ki = ki		#:param ki: float, ki coeff
		self.kd = kd		#:param kd: float, kd coeff
		self.Pterm = 0.0
		self.Iterm = 0.0
		self.Dterm = 0.0
		self.last_error = 0.0
		self.dt = 0.1
		
	def control(self, error):
		"""
		PID main function, given an input, this function will output a control unit
		:param error: float, error term
		:return: float, output control
		"""
		self.Pterm = self.kp * error
		self.Iterm += self.ki*error * self.dt
		self.Dterm += self.kd*error/self.dt
		self.last_error = error
		output = self.Pterm + self.ki * self.Iterm
		return output
	
############################################################################
####                     Pure Pursuit control                           ####
############################################################################
class PurePursuitNode(Node):
	def __init__(self):
		super().__init__('pure_pursuit_node')
		self.declare_parameter('base_frame_id', 'base_link')
		self.declare_parameter('map_frame_id', 'map')
		self.declare_parameter('lookahead_distance', 0.3)
		self.declare_parameter('max_linear_vel', 1.5)
		self.declare_parameter('min_linear_vel', 0.15)
		self.declare_parameter('max_angular_vel', 0.2)
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
		self.declare_parameter('linear_vel', 0.1)
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
		# init variable
		self.current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec/1e9
		self.curr_odom_time = self.current_time
		self.prev_odom_time = self.current_time
		# set up tf
		self.tf_base_lookahead = TransformStamped()
		self.tf_base_lookahead.header.stamp = self.get_clock().now().to_msg()
		self.tf_base_lookahead.header.frame_id = 'base_link'
		self.tf_base_lookahead.child_frame_id = 'lookahead'
		self.tf_base_pose = TransformStamped()
		self.tf_base_pose.header.stamp = self.get_clock().now().to_msg()
		self.tf_base_pose.header.frame_id = 'base_link'
		self.tf_base_pose.child_frame_id = 'path_pose'
		self.tf_map_base = TransformStamped()
		self.tf_map_base.header.stamp = self.get_clock().now().to_msg()
		self.tf_map_base.header.frame_id = self.map_frame_id
		self.tf_map_base.child_frame_id = self.base_frame_id
		# tf listener/broadcaster
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.tf_broadcaster = TransformBroadcaster(self)
		self.broadcast_tf_map_pose_time = self.create_timer(0.02, self.broadcast_tf_map_lookahead_pose)
		# initial state
		self.cmd_vel = Twist()
		self.curr_pose = PoseStamped()
		self.goal_pose = PoseStamped()
		self.reach_point = PoseStamped()
		self.result_status = Int8()
		self.path = Path()
		self.path_init = False
		self.foundIntersection = False
		self.lastFoundIndex = 0
		self.curr_heading = 0.0
		self.goal_heading = 0.0
		self.turn_vel = 0.0
		self.using_rotation = False
		self.txt_info = ""
		# subscription
		self.pose_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
		self.path_subscriber = self.create_subscription(Path, 'path_segment', self.path_callback, 10)
		# publisher
		self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
		self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
	
	def tf_lookup(self,parent,child):
		try:
			return(self.tf_buffer.lookup_transform(parent,child,rclpy.time.Time()))
		except:
			exc_info = sys.exc_info()
			y = traceback.format_exception(*exc_info)
			self.get_logger().error("logerror %s" % y)
			return None
	
	def path_callback(self,path_msg):
		print("Path callback")
		if (path_msg.header.frame_id == self.map_frame_id):
			self.path = path_msg
			self.reach_point.pose.position = self.path.poses[-1].pose.position
			self.path_init = True
			print(f'reach point: x:{self.reach_point.pose.position.x}, y:{self.reach_point.pose.position.y}')

	def odom_callback(self, msg):
		self.curr_pose.pose.position = msg.pose.pose.position
		self.curr_pose.pose.orientation = msg.pose.pose.orientation
		if self.path and self.path_init:
			try:
				tf_map_base = self.tf_lookup(self.map_frame_id, self.base_frame_id)
				self.goal_pose, self.lastFoundIndex, self.turn_vel = self.pure_pursuit(tf_map_base, self.lookahead_distance, self.lastFoundIndex)
				self.cmd_vel.linear.x = self.linear_vel
				self.cmd_vel.angular.z = self.turn_vel[2]*2.5
				# self.goal_publisher.publish(self.goal_pose)
				# self.cmd_vel_publisher.publish(self.cmd_vel)
				print(f'path-angle: {self.turn_vel[	0]:.4f}, robot-angle: {self.turn_vel[1]:.4f}, diff-angle: {self.turn_vel[2]:.4f}, wz: {1.25*self.turn_vel[2]:.4f}')
				# print(f'diff-angle: {self.turn_vel[2]:.4f}, wz: {1.25*self.turn_vel[2]:.4f}')
			except Exception as error:
				print(error)
	
	def pt_to_pt_distance (self, pt1, pt2):
		pt1 = [pt1.pose.position.x, pt1.pose.position.y]
		pt2 = [pt2.pose.position.x, pt2.pose.position.y]
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

	def pure_pursuit(self,tf_map_base, lookAheadDis, LFindex) :
		currentX = tf_map_base.transform.translation.x
		currentY = tf_map_base.transform.translation.y
		goal_return = PoseStamped()
		goalPt = []
		lastFoundIndex = LFindex
		intersectFound = False
		startingIndex = lastFoundIndex
		for i in range (startingIndex, len(self.path.poses)-1): # use for loop to search intersections
			# beginning of line-circle intersection code
			x1 = self.path.poses[i].pose.position.x - currentX
			y1 = self.path.poses[i].pose.position.y - currentY
			x2 = self.path.poses[i+1].pose.position.x - currentX
			y2 = self.path.poses[i+1].pose.position.y - currentY
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
				sol_pt1 = [sol_x1 + currentX, sol_y1 + currentY]
				sol_pt2 = [sol_x2 + currentX, sol_y2 + currentY]
				# end of line-circle intersection code
				minX = min(self.path.poses[i].pose.position.x, self.path.poses[i+1].pose.position.x)
				minY = min(self.path.poses[i].pose.position.y, self.path.poses[i+1].pose.position.y)
				maxX = max(self.path.poses[i].pose.position.x, self.path.poses[i+1].pose.position.x)
				maxY = max(self.path.poses[i].pose.position.y, self.path.poses[i+1].pose.position.y)
				# print(f'min_x:{minX}, min_y:{minY}, max_x:{maxX}, maxY:{maxY}')
				# if one or both of the solutions are in range
				next_pt = [self.path.poses[i+1].pose.position.x,self.path.poses[i+1].pose.position.y]
				if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) or ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
					self.foundIntersection = True
					# if both solutions are in range, check which one is better
					if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
						# make the decision by compare the distance between the intersections and the next point in path
						if self.pt_to_pt_distance2(sol_pt1, next_pt) < self.pt_to_pt_distance2(sol_pt1, next_pt):
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
					if self.pt_to_pt_distance2(goalPt, next_pt) < self.pt_to_pt_distance2([currentX, currentY], next_pt):
						# update lastFoundIndex and exit
						lastFoundIndex = i
						break
					else:
						# in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
						lastFoundIndex = i+1
				# if no solutions are in range
				else:
					self.foundIntersection = False
					# no new intersection found, potentially deviated from the path
					# follow path[lastFoundIndex]
					goalPt = [self.path.poses[lastFoundIndex].pose.position.x, self.path.poses[lastFoundIndex].pose.position.y]
		# obtained goal point, now compute turn vel
		# initialize proportional controller constant
		Kp = 1.0
		# calculate absTargetAngle with the atan2 function
		(_, _, path_angle) = euler_from_quaternion ([self.path.poses[lastFoundIndex].pose.orientation.x,self.path.poses[lastFoundIndex].pose.orientation.y,self.path.poses[lastFoundIndex].pose.orientation.z,self.path.poses[lastFoundIndex].pose.orientation.w])
		(_, _, robot_angle) = euler_from_quaternion([tf_map_base.transform.rotation.x,tf_map_base.transform.rotation.y,tf_map_base.transform.rotation.z,tf_map_base.transform.rotation.w])
		if (self.path.poses[lastFoundIndex].pose.position.z < 0):
			back_robot_angle = robot_angle+math.pi
			if (back_robot_angle > math.pi):
				back_robot_angle = back_robot_angle - (2*math.pi)
			diff_angle = path_angle-back_robot_angle
		else:
			diff_angle = path_angle-robot_angle
		diff_angle = (path_angle - robot_angle + math.pi) % (2 * math.pi) - math.pi
		turnVel = [path_angle, robot_angle, diff_angle]
		goal_return.pose.position.x = goalPt[0]
		goal_return.pose.position.y = goalPt[1]
		goal_return.pose.position.z = 0.0
		goal_return.pose.orientation.x = self.path.poses[lastFoundIndex].pose.orientation.x
		goal_return.pose.orientation.y = self.path.poses[lastFoundIndex].pose.orientation.y
		goal_return.pose.orientation.z = self.path.poses[lastFoundIndex].pose.orientation.z
		goal_return.pose.orientation.w = self.path.poses[lastFoundIndex].pose.orientation.w
		return goal_return, lastFoundIndex, turnVel
	
	def broadcast_tf_map_lookahead_pose(self):
		if (self.path.poses):
			print('broadcast tf map_pose')
			tf_map_pose = TransformStamped()
			tf_map_pose.header.stamp = self.get_clock().now().to_msg()
			tf_map_pose.header.frame_id = 'map'
			tf_map_pose.child_frame_id = 'path_pose'
			tf_map_pose.transform.translation.x = self.path.poses[self.lastFoundIndex].pose.position.x
			tf_map_pose.transform.translation.y = self.path.poses[self.lastFoundIndex].pose.position.y
			tf_map_pose.transform.translation.z = self.path.poses[self.lastFoundIndex].pose.position.z
			tf_map_pose.transform.rotation.x = self.path.poses[self.lastFoundIndex].pose.orientation.x
			tf_map_pose.transform.rotation.y = self.path.poses[self.lastFoundIndex].pose.orientation.y
			tf_map_pose.transform.rotation.z = self.path.poses[self.lastFoundIndex].pose.orientation.z
			tf_map_pose.transform.rotation.w = self.path.poses[self.lastFoundIndex].pose.orientation.w
			self.tf_broadcaster.sendTransform(tf_map_pose)

def main(args=None):
	rclpy.init(args=args)
	node = PurePursuitNode()
	node.control_loop()
	node.destroy_node()
	rclpy.shutdown()

def path_tracking(args=None):
	rclpy.init(args=args)
	node = PurePursuitNode()
	rclpy.spin(node)
	# node.simlulation()
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	path_tracking()
