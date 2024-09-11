#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

import csv
import math
import tf_transformations

class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter('lookahead_distance', 0.3)
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'global_path', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose = None
        # self.path = self.load_path_from_csv('/home/nontanan/robinz_ws/src/path_tracking/csv/recorded_path.csv')
        fileName = '/home/nontanan/robinz_ws/src/path_tracking/csv/waypoint.csv'
        self.path = self.load_path_from_csv(fileName)

        self.path_time = self.create_timer(0.5, self.publish_path)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def pure_pursuit_step(self, path, curr_pose, curr_heading, look_ahead_dist, LFindex):
        current_x = curr_pose.position.x
        current_y = curr_pose.position.y

        # use for loop to search intersections
        lastFoundIndex = LFindex
        intersectFound = False
        startingIndex = lastFoundIndex


    def load_path_from_csv(self, file_path):
        path = Path()
        path.header.frame_id = 'odom'

        with open(file_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
            next(csv_reader)  # Skip header row
            for row in csv_reader:
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.pose.position.x = float(row[0])
                pose.pose.position.y = float(row[1])
                pose.pose.position.z = float(row[2])
                pose.pose.orientation.x = float(row[3])
                pose.pose.orientation.y = float(row[4])
                pose.pose.orientation.z = float(row[5])
                pose.pose.orientation.w = float(row[6])
                path.poses.append(pose)
        
        return path

    def compute_control_command(self):
        if self.current_pose is None or not self.path.poses:
            return None
        
        curr_pose = []
        goal_pose = []
        
        # Find the lookahead point on the path
        lookahead_point = None
        for pose_stamped in self.path.poses:
            pose = pose_stamped.pose
            distance = math.sqrt((pose.position.x - self.current_pose.position.x)**2 +
                                 (pose.position.y - self.current_pose.position.y)**2)
            if distance >= self.lookahead_distance:
                lookahead_point = pose
                break
            curr_pose = [self.current_pose.position.x, self.current_pose.position.y]
            goal_pose = [pose.position.x, pose.position.y]
        
        if lookahead_point is None:
            return None
        
        # Compute the control command
        angle_to_target = math.atan2(lookahead_point.position.y - self.current_pose.position.y,
                                     lookahead_point.position.x - self.current_pose.position.x)
        curr_yaw = self.get_yaw_from_pose(self.current_pose)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.3  # Adjust speed as necessary
        cmd_vel.angular.z =  0.08*(angle_to_target - self.get_yaw_from_pose(self.current_pose))
        # info_txt = "vel: {}, ang: {}, curr_pose: {}, goal_pose: {}".format(cmd_vel.linear.x, angle_to_target, curr_pose, goal_pose)
        info_txt = "curr_pose: {}, curr_heading: {}, track_pose: {}, track_heading: {}".format(curr_pose, curr_yaw, goal_pose, angle_to_target)
        print(info_txt)
        return cmd_vel

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w])
        return yaw
    
    def publish_path(self):
        # Publish the path to a topic
        self.path_publisher.publish(self.path)

    def control_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            control_command = self.compute_control_command()
            if control_command:
                self.cmd_vel_publisher.publish(control_command)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    node.control_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
