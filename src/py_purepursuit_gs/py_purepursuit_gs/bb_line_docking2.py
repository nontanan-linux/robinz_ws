#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Int8
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point, Vector3,TwistStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, copysign, sin, cos, atan2
import traceback
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
from rclpy.time import Time
from autoware_auto_control_msgs.msg import AckermannControlCommand

class MainNode(Node):
    def __init__(self):
        super().__init__('bb_line_docking')

        self.path = Path()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bb_vel = TwistStamped()
        self.ackerman_vel = AckermannControlCommand()
        self.final_vel = Twist()
        self.rate = 30
        self.ros_rate = self.create_rate(self.rate)
        self.ns = self.get_namespace()
        
        ####################
        self.k1 = 1.0  # parallel gain, gain to control theta
        self.k2 = 40.0  # displacement gain, gain to control displacement
        self.bb_ratio = 0.92  # gain to move to next point
        self.base_frame = self.ns + 'base_link'
        self.trailer_frame = self.ns + 'trailer_link'
        self.map_frame_id = 'map'
        self.laser_frame_id = 'scanner_link'
        ####################
        self.ld = 1.5
        self.wheel_base = 1.0
        ####################
        self.first_round = False
        self.idx = 0
        self.bb_error = Point()
        self.goal_reached = False
        self.mode = ''

        self.max_linear_vel = 0.15
        self.max_angular_vel = 0.2

        self.v_acc = 0.0
        self.v_dec = 0.0
        self.acceleration = 0.1
        self.deceleration = 0.5

        self.reach_point1 = False

        # Subscribers
        self.mode_sub = self.create_subscription(String, '/mode', self.mode_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.compute_velocities, 10)
        self.path_sub = self.create_subscription(Path, "/path_seqment", self.path_cb, 10)
    

        # Publishers
        self.pub_bb_vel = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.pub_bb_error = self.create_publisher(Point, '/bb_error', 10)
        self.pub_mission_result = self.create_publisher(Int8, '/mission_result', 10)
        self.pub_control_cmd = self.create_publisher(AckermannControlCommand, '/control/command/docking_cmd', 10)

        self.create_timer(0.1, self.main_loop)

    def path_cb(self, new_path):
        # self.get_logger().info("Number of poses in path: {new_path")
        self.path = new_path
        self.idx = 0
        
        # try:
        #     if new_path.header.frame_id == self.laser_frame_id:
                
        #         self.path = new_path
                
        #     else:
        #         self.get_logger().warn("The path must be published in the map frame! Ignoring path in " + new_path.header.frame_id + " frame!")
        # except Exception as e:
        #     self.get_logger().warn("Error: %s" % traceback.format_exc())

    def points_distance(self, point1, point2):
        return math.sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2))
    
    
    def line_direction(self, point1, point2, point3):
        vector1 = (point2[0] - point1[0], point2[1] - point1[1])  # vector AB
        vector2 = (point3[0] - point1[0], point3[1] - point1[1])  # vector AC

        cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]   # AB cross AC

        if cross_product > 0:
            direction = -1  # left
        elif cross_product < 0:
            direction = 1  # right
        else:
            direction = 0  # on the line

        ab = math.sqrt(pow(vector1[0], 2) + pow(vector1[1], 2))  # size of AB
        ad = (vector1[0] * vector2[0] + vector1[1] * vector2[1]) / pow(ab, 2)
        distance = abs(cross_product) / ab

        return direction, distance, ad

    def nearest_idx(self, path, robot_pose):
        lowest_distance = self.points_distance(path[0].pose.position, robot_pose)
        min_index = 0
        for idx, point in enumerate(path):
            if lowest_distance > self.points_distance(path[idx].pose.position, robot_pose):
                lowest_distance = self.points_distance(path[idx].pose.position, robot_pose)
                min_index = idx
        return min_index

    def robot_pose_lookup(self):
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform(self.map_frame_id, self.base_frame, Time())
                break
            except Exception as e:
                self.get_logger().error("Error looking up transform: %s" % traceback.format_exc())
                continue

        quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )
        _, _, angle = euler_from_quaternion(quaternion)
        return trans.transform.translation, angle

    def trailer_pose_lookup(self):
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform(self.map_frame_id, self.trailer_frame, Time())
                break
            except Exception as e:
                self.get_logger().error("Error looking up trailer transform: %s" % traceback.format_exc())
                continue

        quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )
        _, _, angle = euler_from_quaternion(quaternion)
        return trans.transform.translation, angle

    def compute_velocities(self, odom):
        linear_vel = odom.twist.twist.linear.x

    def mode_cb(self, data):
        self.get_logger().info('Got bb mode')
        if 'uturn' in data.data:
            self.mode = 'uturn'
        elif 'pause' in data.data:
            self.mode = 'pause'
        elif 'bb' in data.data:
            self.mode = 'bb'
        elif 'docking' in data.data:
            self.mode = 'docking'  
            self.idx = 0 
            self.reach_point1 = False
            if self.path.poses:
                self.goal_reached = False
                self.first_round = True
            else:
                self.goal_reached = True
                self.get_logger().warn("Received empty path!")
        else:
            self.mode = ''

    def trailer_vel_to_agv_vel(self, R, L, delta_theta, linear_vel, angular_vel):
        agv_linear_vel = linear_vel * cos(delta_theta) - angular_vel * L * sin(delta_theta)
        agv_angular_vel = (linear_vel * sin(delta_theta) + angular_vel * L * cos(delta_theta)) / (-R)

        if agv_angular_vel != 0:
            agv_linear_vel = agv_linear_vel / max(1, abs(agv_angular_vel / self.max_angular_vel))
            agv_angular_vel = agv_angular_vel / max(1, abs(agv_angular_vel / self.max_angular_vel))

        return agv_linear_vel, agv_angular_vel
    
    def convertToAckerman(self):
        if(abs(self.bb_vel.linear.x) > 10e-6):
            self.ackerman_vel.lateral.steering_tire_angle = atan2(self.wheel_base * self.bb_vel.twist.angular.z , self.bb_vel.twist.linear.x)
        else:
            self.ackerman_vel.lateral.steering_tire_angle = 0.0
        self.ackerman_vel.longitudinal.speed = self.bb_vel.twist.linear.x
        self.pub_control_cmd.publish(self.ackerman_vel)

    def main_loop(self):
        try:
            self.get_logger().info(f"Number of poses in path: {len(self.path.poses)}, Goal reached: {self.goal_reached}")
            if len(self.path.poses) > 1 and not self.goal_reached :
                robot_position = Vector3()
                robot_position.x = 0.0
                robot_position.y = 0.0
                robot_position.z = 0.0
                robot_angle = 0.0  
                # robot_position, robot_angle = self.robot_pose_lookup()
                # trailer_position, trailer_angle = self.robot_pose_lookup()

                if self.first_round and self.path.poses:
                    # self.idx = self.nearest_idx(self.path.poses, robot_position)
                    self.idx = 0
                    
                    # self.get_logger().info(" main loop ")

                    self.first_round = False

                if self.path.poses and self.idx < len(self.path.poses) - 1:
                    point1 = [self.path.poses[self.idx].pose.position.x, self.path.poses[self.idx].pose.position.y]
                    point2 = [self.path.poses[self.idx + 1].pose.position.x, self.path.poses[self.idx + 1].pose.position.y]
                    point3 = [robot_position.x, robot_position.y]

                    direction, distance, ad = self.line_direction(point1, point2, point3)
                    self.get_logger().info(f"AD: {ad} ,{self.idx}")
                    path_orientation = self.path.poses[self.idx].pose.orientation
                    _, _, path_angle = euler_from_quaternion([path_orientation.x, path_orientation.y, path_orientation.z, path_orientation.w])

                    diff_angle = (path_angle - robot_angle)+pi/2-pi/4# cube version
                    # diff_angle = (path_angle - robot_angle)+pi/2# normal version
                    if diff_angle > pi:
                        diff_angle -= 2 * pi
                    elif diff_angle < -pi:
                        diff_angle += 2 * pi

                    # if self.reach_point1:
                    if True:
                        # if self.idx <= len(self.path.poses) - 2 and ad > self.bb_ratio:
                        if ad > self.bb_ratio:
                            self.bb_vel.twist.linear.x = 0.0
                            self.bb_vel.twist.angular.z = 0.0
                            # if not self.goal_reached:
                            # self.mode = ''
                            # self.goal_reached = True
                            # self.pub_mission_result.publish(Int8(data=1))
                            self.get_logger().info("Goal reached")
                            self.pub_bb_error.publish(self.bb_error)
                            self.pub_bb_vel.publish(self.bb_vel)
                            self.convertToAckerman()
                            self.get_logger().info(f"if1")
                            self.idx += 1
                            return
                        elif self.idx == len(self.path.poses) - 2 and ad >= 1.0:
                            self.get_logger().info(f"if2")
                            self.idx += 1

                        if abs(diff_angle) < pi / 3:
                            # self.bb_vel.linear.x = copysign(self.max_linear_vel, self.path.poses[self.idx].pose.position.z)
                            self.bb_vel.twist.linear.x = copysign(self.max_linear_vel, 1)
                        else:
                            self.bb_vel.twist.linear.x = 0.0

                        self.bb_vel.twist.angular.z = self.k1 * diff_angle + self.k2 * distance * direction * abs(self.bb_vel.twist.linear.x)
                        self.get_logger().info(f"angle,distance: {diff_angle} ,{distance}")
                        if abs(self.bb_vel.twist.angular.z) > self.max_angular_vel:
                            pct_angular = abs(self.bb_vel.twist.angular.z) / abs(self.max_angular_vel)
                            self.bb_vel.twist.angular.z = copysign(self.max_angular_vel, self.bb_vel.twist.angular.z)
                            self.bb_vel.twist.linear.x /= pct_angular

                        self.bb_error.x = diff_angle
                        self.bb_error.y = distance
                    else:
                        if point1[0] >= 0.02 or point1[1] >= 0.02:
                            self.bb_vel.twist.linear.x = copysign(self.max_linear_vel,point1[0])
                            self.bb_vel.twist.angular.z = copysign(self.max_angular_vel*0.5 ,point1[0])
                            self.bb_vel.twist.angular.z = copysign(self.max_angular_vel, self.bb_vel.twist.angular.z)
                        else:
                            self.bb_vel.twist.linear.x = 0.0
                            self.bb_vel.twist.angular.z = 0.0
                            self.reach_point1 = True

                else:
                    self.bb_vel.twist.linear.x = 0.0
                    self.bb_vel.twist.angular.z = 0.0
                    if not self.goal_reached:
                        self.mode = ''
                        # self.goal_reached = True
                        self.pub_mission_result.publish(Int8(data=1))
                        self.get_logger().info("Goal reached")
                        self.pub_bb_error.publish(self.bb_error)
                        self.pub_bb_vel.publish(self.bb_vel)
                        self.convertToAckerman()

                # delta_theta = robot_angle - trailer_angle
                # delta_theta = min(max(-pi / 2, delta_theta), pi / 2)
                # self.final_vel.linear.x, self.final_vel.angular.z = self.trailer_vel_to_agv_vel(0.1, 3, delta_theta, self.bb_vel.linear.x, self.bb_vel.angular.z)

                self.pub_bb_error.publish(self.bb_error)
                self.pub_bb_vel.publish(self.bb_vel)
                self.convertToAckerman()
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MainNode()
        # node.main_loop()
        rclpy.spin(node)

    except Exception as e:
        node.get_logger().error("Error: %s" % traceback.format_exc())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
