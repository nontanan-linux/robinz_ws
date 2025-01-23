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
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('max_steering', 1.5707) # 1 rad
        self.declare_parameter('wheel_base', 0.2)
        self.declare_parameter('accelerate', 1.0)
        self.declare_parameter('decelerate', 0.5)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('nearest_start', True)
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
        self.vehicle_model = 1  # 1:differential  2:steering
        
        self.path_resolution = 0.1 #10cm
        self.path_length = 0 #path length
        self.result_status = Int8()
        self.cmd_vel = Twist()
        self.leaving_state = self.appproching_state = self.reach_state = False
        self.current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec/1e9
        self.curr_odom_time = self.current_time
        self.prev_odom_time = self.current_time

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
        self.tf_map_base.header.frame_id = 'map'
        self.tf_map_base.child_frame_id = 'base_link'

        # tf listener/broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.broadcast_tf_map_pose_time = self.create_timer(0.02, self.broadcast_tf_map_lookahead_pose)

        self.current_pose = None
        self.odom_msgs = Odometry()
        fileName = '/home/nontanan/robinz_ws/src/path_tracking/csv/waypoint.csv'
        self.path = self.load_path_from_csv(fileName)
        # self.path_time = self.create_timer(0.5, self.publish_path)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.waypoints = self.load_waypoints_from_csv(fileName)
        self.path_len = []

        #bb variable
        self.bp_ratio = 0.0 #ratio between bb_vel and pp_vel # 0 = 100% pp
        self.bb_idx = 0 #current bb are on this index
        self.k1 = 1.25 #parallel gain, gain to control theta
        self.k2 = 10 #displacement gain, gain to control displacement
        self.bb_ratio = 0.2 #gain to move to next point 
        self.bb_vel = Twist()
        self.leaving_idx = int(2/self.path_resolution)
        self.appproching_idx = int(((self.max_linear_vel*self.max_linear_vel)/(2*self.decelerate))/self.path_resolution)
        
        #purepursuit variable
        self.pp_idx = 0 #current lookahead are on this index
        self.pp_min_ld = 0.4 #min Lookahead distance
        self.pp_max_ld = 0.8 #max Lookahead distance
        self.pp_v_min_ld = 0.8 #velocity for min Lookahead distance
        self.pp_v_max_ld = 1.2 #velocity for min max Lookahead distance
        self.pp_vel = Twist()

        # subscriber
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.odom_subscriber
        self.path_subscriber = self.create_subscription(Path, 'path_segment', self.path_callback, 10)
        self.path_subscriber
        self.path_len_subscriber = self.create_subscription(Int32MultiArray,'path_len',self.path_len_callback,10)
        self.path_len_subscriber
        
        # publisher
        # self.path_publisher = self.create_publisher(Path, 'path_segment', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_result = self.create_publisher(Int8, 'purepursuit_result', 10)
    
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
    
    def tf_lookup(self,parent,child):
        try:
            return(self.tf_buffer.lookup_transform(parent,child,rclpy.time.Time()))
        except:
            exc_info = sys.exc_info()
            y = traceback.format_exception(*exc_info)
            self.get_logger().error("logerror %s" % y)
            return None
    
    def path_callback(self,new_path):
        if (new_path.header.frame_id == self.map_frame_id):
            self.path = new_path
            if (new_path.poses and self.nearest_start):
                self.bb_idx = self.pp_idx = self.find_nearest_idx(new_path)
                self.path_length = len(new_path.poses)
            elif (new_path.poses and not self.nearest_start):
                self.bb_idx = self.pp_idx = 0
                self.path_length = len(new_path.poses)
            else:
                self.get_logger().warn('Receive empty path')
                return

            if (self.bb_idx == len(new_path.poses)-1):
                self.leaving_state = self.appproching_state = self.reach_state = True
                self.get_logger().warn('at the end of path')
            else:
                self.leaving_state = self.appproching_state = self.reach_state = False
            print(f'index: {self.bb_idx}, path length: {self.path_length}')
        else:
            self.get_logger().info('The path must be published in the map frame')
        print('got new path')
    
    def get_distance(self, curr_pose, goal_pose):
        dx = goal_pose.position.x - curr_pose.position.x
        dy = goal_pose.position.y - curr_pose.position.y
        return math.sqrt(dx**2 +dy**2)
    
    def distance(self, pose1, pose2):
        return math.sqrt(math.pow(pose1.x - pose2.x,2) + math.pow(pose1.y - pose2.y,2))
    
    def find_nearest_idx(self, path):
        try:
            tf_map_base = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn('Could not get transform from map to base_link')
            return 0

        nearest_distance = self.distance(path.poses[0].pose.position,tf_map_base.transform.translation) 
        nearest_idx = 0
        for i in range(len(path.poses)):
            if nearest_distance > self.distance(path.poses[i].pose.position,tf_map_base.transform.translation):
                nearest_distance = self.distance(path.poses[i].pose.position,tf_map_base.transform.translation)
                nearest_idx = i
        # print('nearest idx:',nearest_idx)
        return nearest_idx
    
    def path_len_callback(self,data):
        self.path_len = data.data
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.path and self.path_len:
            self.curr_odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            tf_map_base = self.tf_lookup('map', 'base_link')
            if not tf_map_base: return
            pp_linear_vel, pp_angular_vel = self.cal_pp_vel(msg, tf_map_base)
            bb_linear_vel, bb_angular_vel, diff = self.cal_bb_vel(tf_map_base)
            v_accel = self.cal_v_accelerate(msg)
            v_decel = self.cal_v_decelerate()  
            # print(pp_linear_vel,v_accel)
            if self.path.poses and not self.reach_state:
                self.cmd_vel.linear.x = max(min(min((self.bp_ratio*bb_linear_vel+((1-self.bp_ratio)*pp_linear_vel)),v_accel),v_decel),self.min_linear_vel)
            else:
                self.cmd_vel.linear.x = 0.0
            if self.vehicle_model == 1:
                self.cmd_vel.angular.z = self.bp_ratio*bb_angular_vel + ((1-self.bp_ratio)*pp_angular_vel)
            elif self.vehicle_model == 2:
                self.cmd_vel.angular.z = bb_angular_vel
                # self.cmd_vel.angular.z = self.angular_vel_to_steer(self.cmd_vel.linear.x,self.cmd_vel.angular.z)
            # print(f'accel: {v_accel}, diff-angle: {diff_angle}, w.z: {self.cmd_vel.angular.z}')
            print("path-angle: {:.4f}, robot-angle: {:.4f}, diff-angle: {:.4f}, wz: {:.4f}".format(diff[0], diff[1], diff[2], self.cmd_vel.angular.z))
            # print("diff-angle: {:.4f}, wz: {:.4f}".format(diff[0], diff[1], diff[2], self.cmd_vel.angular.z))
            self.cmd_vel_publisher.publish(self.cmd_vel)
            if self.bb_idx >= self.leaving_idx and len(self.path_len) == 1 and not self.leaving_state:
                self.result_status.data = 21
                self.pub_result.publish(self.result_status)
                self.leaving_state = True
            elif self.bb_idx >= ((self.path_len[0]-1) - self.appproching_idx) and len(self.path_len) == 1 and not self.appproching_state:
                self.result_status.data = 22
                self.pub_result.publish(self.result_status)
                self.appproching_state = True
            elif self.bb_idx >= (self.path_len[0]-1) and not self.reach_state:
                self.result_status.data = 23
                self.pub_result.publish(self.result_status)
                self.reach_state = True
            self.prev_odom_time = self.curr_odom_time
    
    def bb_line_direction(self, point1, point2, point3):
        vector1 = (point2[0] - point1[0], point2[1] - point1[1])  # vector AB
        vector2 = (point3[0] - point1[0], point3[1] - point1[1])  # vector AC
        cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]   # AB cross AC
        direction = (cross_product < 0) - (cross_product > 0) 
        ab = math.sqrt(pow(vector1[0], 2) + pow(vector1[1], 2))  # size of AB
        ad = (vector1[0] * vector2[0] + vector1[1] * vector2[1]) / pow(ab, 2)
        distance = abs(cross_product) / ab
        return direction, distance, ad
    
    def cal_bb_vel(self,tf_map_base):
        bb_linear_vel = bb_angular_vel = 0
        diff = [0.0,0.0,0.0]
        if self.path.poses and self.bb_idx < len(self.path.poses)-1:
            point1 = [self.path.poses[self.bb_idx].pose.position.x,self.path.poses[self.bb_idx].pose.position.y]
            point2 = [self.path.poses[self.bb_idx+1].pose.position.x,self.path.poses[self.bb_idx+1].pose.position.y]
            point3 = [tf_map_base.transform.translation.x,tf_map_base.transform.translation.y]
            direction,distance,ad = self.bb_line_direction(point1,point2,point3)
            (_, _, path_angle) = euler_from_quaternion ([self.path.poses[self.bb_idx].pose.orientation.x,self.path.poses[self.bb_idx].pose.orientation.y,self.path.poses[self.bb_idx].pose.orientation.z,self.path.poses[self.bb_idx].pose.orientation.w])
            (_, _, robot_angle) = euler_from_quaternion([tf_map_base.transform.rotation.x,tf_map_base.transform.rotation.y,tf_map_base.transform.rotation.z,tf_map_base.transform.rotation.w])
            if (self.path.poses[self.bb_idx].pose.position.z < 0):
                back_robot_angle = robot_angle+math.pi
                if (back_robot_angle > math.pi):
                    back_robot_angle = back_robot_angle - (2*math.pi)
                diff_angle = path_angle-back_robot_angle
            else:
                diff_angle = path_angle-robot_angle
            diff_angle = (path_angle - robot_angle + math.pi) % (2 * math.pi) - math.pi
            diff = [path_angle, robot_angle, diff_angle]
            # print('diff_angle,robot_angle,path_angle',robot_angle-path_angle,robot_angle,path_angle)

            if self.bb_idx < len(self.path.poses)-2 and ad > self.bb_ratio:
                self.bb_idx = self.bb_idx+1
            elif self.bb_idx == len(self.path.poses)-2 and ad >= 1:
                self.bb_idx = self.bb_idx+1
            if abs(diff_angle) < math.pi/6:
                # bb_linear_vel = math.copysign(self.v_max,self.path.poses[self.bb_idx].pose.position.z)
                bb_linear_vel = self.path.poses[self.bb_idx].pose.position.z
            else:
                bb_linear_vel = 0.0
            # bb_angular_vel = self.k1*diff_angle+self.k2*distance*direction*abs(bb_linear_vel)
            bb_angular_vel = self.k1*diff_angle
            # if abs(bb_angular_vel) > self.max_angular_vel:
            #     pct_angular = abs(bb_angular_vel)/abs(self.max_angular_vel)
            #     bb_angular_vel = math.copysign(self.max_angular_vel,bb_angular_vel)
            #     bb_linear_vel = bb_linear_vel/pct_angular
        else:
            bb_linear_vel = bb_angular_vel = 0.0
        return(bb_linear_vel,bb_angular_vel, diff)
    
    def cal_pp_vel(self,odom,tf_map_base):
        self.cal_pp_index_and_lookahead(tf_map_base,odom)
        # odom_time = (odom.header.stamp.sec+odom.header.stamp.nanosec/1e9);
        pp_result = Int8()
        try:
            self.tf_base_pose = self.tf_buffer.lookup_transform('base_link','path_pose',rclpy.time.Time())
        except:
            print ('cannot_lookup_base_link_to_path_pose')
            return (0,0)
        if (self.path.poses and self.pp_idx >= len(self.path.poses)-1 and self.tf_base_pose.transform.translation.x <= self.tolerance):
            pp_linear_vel = pp_angular_vel = 0
        elif (self.path.poses and not self.reach_state ):
            pp_result.data = 0
            yt = self.tf_base_pose.transform.translation.y
            ld_2 = self.ld * self.ld
            steering_angle = max(min( math.atan2(2 * yt * self.wheel_base, ld_2), self.max_steering), -self.max_steering)
            if (self.path.poses[self.bb_idx].pose.position.z > 0):
                v = self.path.poses[self.bb_idx].pose.position.z
                pp_angular_vel = float(min( 2*v/ ld_2 * yt, self.max_angular_vel))
                pp_linear_vel = float(v)
            elif (self.path.poses[self.bb_idx].pose.position.z < 0):
                v = -self.path.poses[self.bb_idx].pose.position.z
                pp_angular_vel = float(min( -2*v/ ld_2 * yt, self.max_angular_vel))
                pp_linear_vel = float(-v)
            else:
                pp_angular_vel = float(0)
                pp_linear_vel = float(0)
        else:
            pp_linear_vel = float(0)
            pp_angular_vel = float(0)

            if(pp_result != 3):
                pp_result.data = 3
        return(pp_linear_vel,pp_angular_vel)
    
    def cal_pp_index_and_lookahead(self,tf_map_base,odom):
        self.ld = min((max((odom.twist.twist.linear.x-self.pp_v_min_ld)/(self.pp_v_max_ld-self.pp_v_min_ld),0)),1)*(self.pp_max_ld-self.pp_min_ld)+self.pp_min_ld
        for i in range(self.pp_idx,len(self.path.poses)):
            if self.distance(self.path.poses[i].pose.position,tf_map_base.transform.translation) > self.ld :
                self.pp_idx = i   
                break

    def cal_v_accelerate(self,new_odom):
        if self.path.poses:
            return math.copysign((abs(new_odom.twist.twist.linear.x)+(self.accelerate*(self.curr_odom_time-self.prev_odom_time))),self.path.poses[self.bb_idx].pose.position.z) #V=U+aT
        else:
            return 0
        
    def cal_v_decelerate(self):
        # return math.copysign((abs(new_odom.twist.twist.linear.x)-(self.decelerate*(self.odom_time-self.prev_odom_time))),new_odom.twist.twist.linear.x) #V=U+aT
        return math.sqrt(2*self.decelerate*abs(((self.path_length-1)-self.bb_idx)*self.path_resolution))  #U = sqrt(2(-a)s)
    
    def angular_vel_to_steer(self,linear_vel,angular_vel):
        return math.atan2(self.wheel_base * angular_vel, linear_vel) if linear_vel != 0 else 0

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
            self.get_logger().error('No current pose or path subscript')
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
        self.get_logger().debug(info_txt)
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
    
    def broadcast_tf_map_lookahead_pose(self):
        # print('broadcast tf map_pose')
        if (self.path.poses):
            tf_map_pose = TransformStamped()
            tf_map_pose.header.stamp = self.get_clock().now().to_msg()
            tf_map_pose.header.frame_id = 'map'
            tf_map_pose.child_frame_id = 'path_pose'
            tf_map_pose.transform.translation.x = self.path.poses[self.pp_idx].pose.position.x
            tf_map_pose.transform.translation.y = self.path.poses[self.pp_idx].pose.position.y
            tf_map_pose.transform.translation.z = self.path.poses[self.pp_idx].pose.position.z
            tf_map_pose.transform.rotation.x = self.path.poses[self.pp_idx].pose.orientation.x
            tf_map_pose.transform.rotation.y = self.path.poses[self.pp_idx].pose.orientation.y
            tf_map_pose.transform.rotation.z = self.path.poses[self.pp_idx].pose.orientation.z
            tf_map_pose.transform.rotation.w = self.path.poses[self.pp_idx].pose.orientation.w
            self.tf_broadcaster.sendTransform(tf_map_pose)

    def control_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            control_command = self.compute_control_command()
            if control_command:
                self.cmd_vel_publisher.publish(control_command)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    # node.control_loop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
