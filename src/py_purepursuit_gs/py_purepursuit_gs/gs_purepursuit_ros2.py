# regular import
import rclpy
import math
import numpy as np
from rclpy.node import Node

# import ros_msg
from std_msgs.msg import String, Int8
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist,TransformStamped

# import tf2 
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster

def euler_from_quaternion(quaternion):
	x = quaternion.x
	y = quaternion.y
	z = quaternion.z
	w = quaternion.w

	sinr_cosp = 2 * (w * x + y * z)
	cosr_cosp = 1 - 2 * (x * x + y * y)
	roll = np.arctan2(sinr_cosp, cosr_cosp)

	sinp = 2 * (w * y - z * x)
	pitch = np.arcsin(sinp)

	siny_cosp = 2 * (w * z + x * y)
	cosy_cosp = 1 - 2 * (y * y + z * z)
	yaw = np.arctan2(siny_cosp, cosy_cosp)

	return roll, pitch, yaw

class Purepursuit_ros2(Node):

	def __init__(self):
		super().__init__('purepursuit_ros2')
		self.declare_parameter('my_name', 'AGV1')
		self.my_name = self.get_parameter('my_name').get_parameter_value().string_value
		# agv_parameter
		self.steering_max = 1.5707 #rad
		self.v_max = 1 #Maximum velocity
		self.v_min = 0.15 #Minimum velocity
		self.w_max = 1 #Maximum angular velocity
		self.map_frame_id = 'map'
		self.ld = 2 #Lookahead distance
		self.goal_tolerance = 0.05
		self.accelerate = 0.2
		self.decelerate = 0.2
		self.L = 1.3 #wheelbase

		# subscriber
		self.sub_path = self.create_subscription(Path,'path_segment',self.receive_path,10)
		self.sub_path
		self.sub_odom = self.create_subscription(Odometry,'odom',self.receive_odom,10)
		self.sub_odom

		#publisher
		self.pub_result = self.create_publisher(Int8, 'pp_result', 10)
		self.pub_vel = self.create_publisher(Twist , 'cmd_vel', 10)

		# tf2
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.tf_broadcaster = TransformBroadcaster(self)
		self.broadcast_tf_map_pose_time = self.create_timer(0.02, self.broadcast_tf_map_pose)

		# general_parameter
		self.goal_reach = False
		self.path = Path()
		self.path_idx = 0

		start_time = self.get_clock().now().to_msg()
		self.prev_odom_time = start_time.sec + start_time.nanosec/1e9

		self.tf_base_lookahead = TransformStamped()
		self.tf_base_lookahead.header.stamp = self.get_clock().now().to_msg()
		self.tf_base_lookahead.header.frame_id = 'base_link'+ "_" + self.my_name
		self.tf_base_lookahead.child_frame_id = 'lookahead'+ "_" + self.my_name

		self.tf_base_pose = TransformStamped()
		self.tf_base_pose.header.stamp = self.get_clock().now().to_msg()
		self.tf_base_pose.header.frame_id = 'base_link'+ "_" + self.my_name
		self.tf_base_pose.child_frame_id = 'path_pose'+ "_" + self.my_name

		self.tf_map_base = TransformStamped()
		self.tf_map_base.header.stamp = self.get_clock().now().to_msg()
		self.tf_map_base.header.frame_id = 'map'
		self.tf_map_base.child_frame_id = 'base_link'+ "_" + self.my_name
		

	def receive_path(self,new_path):
		if (new_path.header.frame_id == self.map_frame_id):
			self.path = new_path
			if (new_path.poses):
				self.path_idx = self.find_nearest_idx(new_path)
			else:
				self.get_logger().info('Receive empty path')
				return

			if (self.path_idx == len(new_path.poses)-1):
				self.goal_reach = True
			else:
				self.goal_reach = False
			print(self.path_idx)
		else:
			self.get_logger().info('The path must be published in the map frame')

		print('got path')

	def receive_odom(self,new_odom):
		odom_time = (new_odom.header.stamp.sec+new_odom.header.stamp.nanosec/1e9);
		cmd_vel = Twist()
		pp_result = Int8()
		
		

		
		
		if (abs(new_odom.twist.twist.linear.x) > 0.6):
			self.ld = 2
		else:
			self.ld = 2
		
		try:
			self.tf_map_base = self.tf_buffer.lookup_transform('map','base_link'+ "_" + self.my_name,rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=2.0))
			# print('got_self.tf_map_base_link')
		except Exception as e:
			print(e)
			return
		for i in range(self.path_idx,len(self.path.poses)):
			if self.distance(self.path.poses[i].pose.position,self.tf_map_base.transform.translation) > self.ld :
				self.path_idx = i	
				try:
					self.tf_base_pose = self.tf_buffer.lookup_transform('base_link'+ "_" + self.my_name,'path_pose'+ "_" + self.my_name,rclpy.time.Time())
				except:
					print ('lookup_base_link_path_pose')
					return
				self.tf_base_lookahead.transform = self.tf_base_pose.transform
				self.tf_broadcaster.sendTransform(self.tf_base_lookahead)
				print(len(self.path.poses),self.path_idx)
				break
	

		if (self.path.poses and self.path_idx >= len(self.path.poses)-1):
			try:
				self.tf_base_pose = self.tf_buffer.lookup_transform('base_link'+ "_" + self.my_name,'path_pose'+ "_" + self.my_name,rclpy.time.Time())
			except:
				print ('lookup_base_link_path_pose')
				return
			if (self.tf_base_pose.transform.translation.x <= self.goal_tolerance):
				print("self.tf_base_pose",self.tf_base_pose.transform.translation.x)
				self.goal_reach = True
				print('goal reach')
				self.path = Path()
			else:
				print("self.tf_base_pose",self.tf_base_pose.transform.translation.x)
				(_,_,yaw) = euler_from_quaternion(self.tf_base_pose.transform.rotation)
				m_goal = math.tan(yaw)
				l_goal = self.tf_base_pose.transform.translation.y - m_goal*self.tf_base_pose.transform.translation.x
				a = 1 + m_goal * m_goal
				b = 2 * l_goal * m_goal
				temp_ld = self.ld
				c = l_goal * l_goal - temp_ld * temp_ld
				print('yaw:', yaw)
				# print('c',b*b - 4*a*c,yaw,l_goal,self.tf_base_pose.transform.translation.y,self.tf_base_pose.transform.translation.x)
				while (b*b - 4*a*c < 0 ):
					temp_ld = temp_ld+0.01
					c = l_goal * l_goal - temp_ld*temp_ld
					print('c in while',b*b - 4*a*c,yaw,l_goal,self.tf_base_pose.transform.translation.y,self.tf_base_pose.transform.translation.x)
				D = math.sqrt(b*b - 4*a*c)
				x_ld = (-b + math.copysign(D,1)) / (2*a)
				if(self.tf_base_pose.transform.translation.z < 0):
					x_ld = (-b + math.copysign(D,-1)) / (2*a)
				y_ld = m_goal * x_ld + l_goal

				self.tf_base_lookahead = TransformStamped()
				self.tf_base_lookahead.header.stamp = self.get_clock().now().to_msg()
				self.tf_base_lookahead.header.frame_id = 'base_link'+ "_" + self.my_name
				self.tf_base_lookahead.child_frame_id = 'lookahead'+ "_" + self.my_name
				self.tf_base_lookahead.transform.translation.x = x_ld
				self.tf_base_lookahead.transform.translation.y = y_ld
				self.tf_base_lookahead.transform.translation.z = self.tf_base_pose.transform.translation.z
				self.tf_base_lookahead.transform.rotation = self.tf_base_pose.transform.rotation
				self.tf_broadcaster.sendTransform(self.tf_base_lookahead)
				print('self.tf_base_lookahead',self.tf_base_lookahead.transform.translation.x,self.tf_base_lookahead.transform.translation.y)

		if (not self.goal_reach and self.path.poses):

			
			pp_result.data = 0
			yt = self.tf_base_lookahead.transform.translation.y
			ld_2 = self.ld * self.ld

			steering_angle = max(min( math.atan2(2 * yt * self.L, ld_2), self.steering_max), -self.steering_max)
			v_accelerate = abs(new_odom.twist.twist.linear.x)+(self.accelerate*(odom_time-self.prev_odom_time)) #V=U+aT
			v_decelerate = math.sqrt(2*self.decelerate*abs(self.distance(self.path.poses[-1].pose.position, self.tf_map_base.transform.translation)- self.goal_tolerance))  #U = sqrt(2(-a)s)
			v = max(min(v_decelerate,v_accelerate,self.v_max),self.v_min)
			print(v_decelerate,v_accelerate,self.v_max,self.v_min)
			if (self.tf_base_pose.transform.translation.z > 0):
				v = min(v,self.tf_base_pose.transform.translation.z)
				cmd_vel.angular.z = float(min( 2*v/ ld_2 * yt, self.w_max))
				cmd_vel.linear.x = float(v)
			elif (self.tf_base_pose.transform.translation.z < 0):
				v = min(v,-self.tf_base_pose.transform.translation.z)
				cmd_vel.angular.z = float(min( -2*v/ ld_2 * yt, self.w_max))
				cmd_vel.linear.x = float(-v)
			else:
				cmd_vel.angular.z = float(0)
				cmd_vel.linear.x = float(0)

		elif (not self.goal_reach and not self.path.poses):
			self.tf_base_lookahead = TransformStamped()
			self.tf_base_lookahead.header.stamp = self.get_clock().now().to_msg()
			self.tf_base_lookahead.header.frame_id = 'base_link'+ "_" + self.my_name
			self.tf_base_lookahead.child_frame_id = 'lookahead'+ "_" + self.my_name
			self.tf_base_lookahead.transform.rotation.w = float(1)

		else:
			self.tf_base_lookahead = TransformStamped()
			self.tf_base_lookahead.header.stamp = self.get_clock().now().to_msg()
			self.tf_base_lookahead.header.frame_id = 'base_link'+ "_" + self.my_name
			self.tf_base_lookahead.child_frame_id = 'lookahead'+ "_" + self.my_name
			self.tf_base_lookahead.transform.rotation.w = float(1)
			cmd_vel.linear.x = float(0)
			cmd_vel.angular.z = float(0)

			if(pp_result != 3):
				self.pub_vel.publish(cmd_vel);
				pp_result.data = 3;
				self.pub_result.publish(pp_result);

		self.tf_base_lookahead.header.stamp = self.get_clock().now().to_msg()
		self.tf_broadcaster.sendTransform(self.tf_base_lookahead)
		# print('pub_cmd_vel')
		self.pub_vel.publish(cmd_vel)
		self.prev_odom_time = odom_time
								   
	def find_nearest_idx(self, new_path):
		try:
			self.tf_map_base = self.tf_buffer.lookup_transform('map','base_link'+ "_" + self.my_name,rclpy.time.Time())
		except TransformException as ex:
			self.get_logger().info('Could not get transform from map to base_link')
			return -1
		nearest_distance = self.distance(new_path.poses[0].pose.position,self.tf_map_base.transform.translation) 
		nearest_idx = 0
		for i in range(len(new_path.poses)):
			if nearest_distance > self.distance(new_path.poses[i].pose.position,self.tf_map_base.transform.translation):
				nearest_distance = self.distance(new_path.poses[i].pose.position,self.tf_map_base.transform.translation)
				nearest_idx = i
		print(nearest_idx)
		return nearest_idx

	def distance(self, pose1, pose2):
		return math.sqrt(math.pow(pose1.x - pose2.x,2) + math.pow(pose1.y - pose2.y,2))

	def broadcast_tf_map_pose(self):
		# print('broadcast tf map_pose')
		if (self.path.poses):
			tf_map_pose = TransformStamped()
			tf_map_pose.header.stamp = self.get_clock().now().to_msg()
			tf_map_pose.header.frame_id = 'map'
			tf_map_pose.child_frame_id = 'path_pose'+ "_" + self.my_name
			tf_map_pose.transform.translation.x = self.path.poses[self.path_idx].pose.position.x
			tf_map_pose.transform.translation.y = self.path.poses[self.path_idx].pose.position.y
			tf_map_pose.transform.translation.z = self.path.poses[self.path_idx].pose.position.z
			tf_map_pose.transform.rotation.x = self.path.poses[self.path_idx].pose.orientation.x
			tf_map_pose.transform.rotation.y = self.path.poses[self.path_idx].pose.orientation.y
			tf_map_pose.transform.rotation.z = self.path.poses[self.path_idx].pose.orientation.z
			tf_map_pose.transform.rotation.w = self.path.poses[self.path_idx].pose.orientation.w
			self.tf_broadcaster.sendTransform(tf_map_pose)

	
def main(args=None):
	rclpy.init(args=args)

	purepursuit_ros2 = Purepursuit_ros2()

	rclpy.spin(purepursuit_ros2)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	purepursuit_ros2.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()