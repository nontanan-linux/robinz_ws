# regular import
import rclpy
import math
import numpy as np
import sys, traceback
from rclpy.node import Node

# import ros_msg
from std_msgs.msg import String, Int8, Int32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist,TransformStamped
from builtin_interfaces.msg import Time

# import tf2 
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

print(f'numpy version: {np.__version__}')

class BB_purepursuit_ros2(Node):

    def __init__(self):
        super().__init__('bb_purepursuit_ros2')
        
        #general variable
        self.declare_parameter('agv_name', '')
        self.agv_name = self.get_parameter('agv_name').get_parameter_value().string_value
        self.map_frame_id = 'map'
        # self.base_frame_id = self.agv_name+'base_link'
        self.base_frame_id = 'base_link'
        self.path = Path()
        self.path_len = []
        self.nearest_start = True # start from nearest point in path / False = start from the first point in path
        self.vehicle_model = 1  # 1:differential  2:steering

        self.steering_max = 1.5707 #rad
        self.wheel_base = 0.6
        self.L = 0.14 #wheelbase
        self.accelerate = 1.0
        self.decelerate = 0.5
        self.goal_tolerance = 0.05
        self.v_min = 0.15 #Minimum linear velocity
        self.v_max = 0.8 #Maximum linear velocity   
        self.w_min = 0.1 #Minimum angular velocity
        self.w_max = 1.2 #Maximum angular velocity
        self.path_resolution = 0.1 #10cm
        self.path_length = 0 #path length
        self.bp_ratio = 0.8 #ratio between bb_vel and pp_vel # 0 = 100% pp 0.3
        self.result_status = Int8()
        self.cmd_vel = Twist()
        self.leaving_state = self.appproching_state = self.reach_state = False
        start_time = self.get_clock().now().to_msg()
        self.odom_time = start_time.sec + start_time.nanosec/1e9
        self.prev_odom_time = start_time.sec + start_time.nanosec/1e9
        
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

        
        
        #bb variable
        self.bb_idx = 0 #current bb are on this index
        self.k1 = 1 #parallel gain, gain to control theta
        self.k2 = 10 #displacement gain, gain to control displacement
        self.bb_ratio = 0.2 #gain to move to next point 
        self.bb_vel = Twist()
        self.leaving_idx = int(2/self.path_resolution)
        self.appproching_idx = int(((self.v_max*self.v_max)/(2*self.decelerate))/self.path_resolution)
        
        #purepursuit variable
        self.pp_idx = 0 #current lookahead are on this index
        self.pp_min_ld = 0.2 #min Lookahead distance 0.4-0.8
        self.pp_max_ld = 0.6 #max Lookahead distance
        self.pp_v_min_ld = 0.8 #velocity for min Lookahead distance
        self.pp_v_max_ld = 1.2 #velocity for min max Lookahead distance
        self.pp_vel = Twist()

        #subscriber
        self.sub_odom = self.create_subscription(Odometry,'odom',self.receive_odom,10)
        self.sub_odom
        self.sub_path = self.create_subscription(Path,'path_segment',self.receive_path,10)
        self.sub_path
        self.sub_path_len = self.create_subscription(Int32MultiArray,'path_len',self.receive_path_len,10)
        self.sub_path_len

        #publisher
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_result = self.create_publisher(Int8, 'purepursuit_result', 10)

    def tf_lookup(self,parent,child):
        try:
            return(self.tf_buffer.lookup_transform(parent,child,rclpy.time.Time()))
            # self.tf_map_base = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time())
            # (_, _, angle) = euler_from_quaternion ([self.tf_map_base.transform.rotation.x,self.tf_map_base.transform.rotation.y,self.tf_map_base.transform.rotation.z,self.tf_map_base.transform.rotation.w])
            # return self.tf_map_base.transform
        except:
            exc_info = sys.exc_info()
            y = traceback.format_exception(*exc_info)
            self.get_logger().error("logerror %s" % y)
            return None

    def tf_lookup_loop(self,parent,child):
        while True:
            try:
                return(self.tf_buffer.lookup_transform(parent,child,rclpy.time.Time()))
                # self.tf_map_base = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time())
                # (_, _, angle) = euler_from_quaternion ([self.tf_map_base.transform.rotation.x,self.tf_map_base.transform.rotation.y,self.tf_map_base.transform.rotation.z,self.tf_map_base.transform.rotation.w])
                # return self.tf_map_base.transform
            except:
                exc_info = sys.exc_info()
                y = traceback.format_exception(*exc_info)
                self.get_logger().error("logerror %s" % y)
                continue
        
    def receive_path(self,new_path):
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

    

    def distance(self, pose1, pose2):
        return math.sqrt(math.pow(pose1.x - pose2.x,2) + math.pow(pose1.y - pose2.y,2))

    def find_nearest_idx(self, new_path):
        try:
            tf_map_base = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn('Could not get transform from map to base_link')
            return 0

        nearest_distance = self.distance(new_path.poses[0].pose.position,tf_map_base.transform.translation) 
        nearest_idx = 0
        for i in range(len(new_path.poses)):
            if nearest_distance > self.distance(new_path.poses[i].pose.position,tf_map_base.transform.translation):
                nearest_distance = self.distance(new_path.poses[i].pose.position,tf_map_base.transform.translation)
                nearest_idx = i
        # print('nearest idx:',nearest_idx)
        return nearest_idx

    def receive_path_len(self,data):
        self.path_len = data.data

    def receive_odom(self,new_odom):
        if self.path and self.path_len:
            self.odom_time = new_odom.header.stamp.sec+new_odom.header.stamp.nanosec/1e9
            # self.get_logger().info('I heard: "%s"' % new_odom.twist.twist.linear.x)
            tf_map_base = self.tf_lookup('map','base_link')
            if not tf_map_base: return

            pp_linear_vel,pp_angular_vel = self.cal_pp_vel(new_odom,tf_map_base)
            bb_linear_vel,bb_angular_vel = self.cal_bb_vel(tf_map_base)
            v_accel = self.cal_v_accelerate(new_odom)
            v_decel = self.cal_v_decelerate()

            print(f'linear vel: {pp_linear_vel}, accel: {v_accel}')

            if self.path.poses and not self.reach_state:
                self.cmd_vel.linear.x = max(min(min((self.bp_ratio*bb_linear_vel+((1-self.bp_ratio)*pp_linear_vel)),v_accel),v_decel),self.v_min)
            else:
                self.cmd_vel.linear.x = 0.0

            if self.vehicle_model == 1:
                self.cmd_vel.angular.z = self.bp_ratio*bb_angular_vel + ((1-self.bp_ratio)*pp_angular_vel)
            elif self.vehicle_model == 2:
                self.cmd_vel.angular.z = self.angular_vel_to_steer(self.cmd_vel.linear.x,self.cmd_vel.angular.z)
            self.pub_vel.publish(self.cmd_vel)

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

            self.prev_odom_time = self.odom_time

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
            bb_angular_vel = self.k1*diff_angle+self.k2*distance*direction*abs(bb_linear_vel)
            # bb_angular_vel = self.k1*diff_angle+self.k2*distance*direction
            if abs(bb_angular_vel) > self.w_max:
                pct_angular = abs(bb_angular_vel)/abs(self.w_max)
                bb_angular_vel = math.copysign(self.w_max,bb_angular_vel)
                bb_linear_vel = bb_linear_vel/pct_angular
            # bb_angular_vel = self.k1*diff_angle*(-1.0)+self.k2*distance*bb_linear_vel*(sin(diff_angle)/diff_angle)

            # self.bb_error.x = diff_angle
            # self.bb_error.y = distance
        else:
            bb_linear_vel = bb_angular_vel = 0.0
            # if not self.reach_state:
            #     self.reach_state = True
            # #     self.pub_mission_result.publish(1)
            #     print("reach")
        # delta_theta = robot_angle-trailer_angle
        # delta_theta = trailer_angle-robot_angle
        # delta_theta = min(max(-math.pi/2,delta_theta),math.pi/2)
        # self.final_vel.linear.x,self.final_vel.angular.z = self.trailer_vel_to_agv_vel(0.1,3,delta_theta,bb_linear_vel,bb_angular_vel)
        # print(self.final_vel.linear.x,self.final_vel.angular.z,robot_position,robot_angle,self.bb_error.x,self.bb_error.y)
        # self.pub_bb_error.publish(self.bb_error)
        # self.pub_bb_vel.publish(self.bb_vel)
        return(bb_linear_vel,bb_angular_vel)

    def cal_pp_vel(self,odom,tf_map_base):
        self.cal_pp_index_and_lookahead(tf_map_base,odom)
        # odom_time = (odom.header.stamp.sec+odom.header.stamp.nanosec/1e9);
        pp_result = Int8()
        try:
            self.tf_base_pose = self.tf_buffer.lookup_transform('base_link','path_pose',rclpy.time.Time())
        except:
            print ('cannot_lookup_base_link_to_path_pose')
            return (0,0)
        if (self.path.poses and self.pp_idx >= len(self.path.poses)-1 and self.tf_base_pose.transform.translation.x <= self.goal_tolerance):
                # print("self.tf_base_pose",self.tf_base_pose.transform.translation.x)
            # self.reach_state = True
            # print('goal reach')
            # self.path = Path()
            pp_linear_vel = pp_angular_vel = 0
                # pp_angular_vel = 0
                # print("self.tf_base_pose",self.tf_base_pose.transform.translation.x)
                # (_,_,yaw) = euler_from_quaternion(self.tf_base_pose.transform.rotation)
                # m_goal = math.tan(yaw)
                # l_goal = self.tf_base_pose.transform.translation.y - m_goal*self.tf_base_pose.transform.translation.x
                # a = 1 + m_goal * m_goal
                # b = 2 * l_goal * m_goal
                # temp_ld = self.ld
                # c = l_goal * l_goal - temp_ld * temp_ld
                # # print('yaw:', yaw)
                # # print('c',b*b - 4*a*c,yaw,l_goal,self.tf_base_pose.transform.translation.y,self.tf_base_pose.transform.translation.x)
                # while (b*b - 4*a*c < 0 ):
                #     temp_ld = temp_ld+0.01
                #     c = l_goal * l_goal - temp_ld*temp_ld
                #     # print('c in while',b*b - 4*a*c,yaw,l_goal,self.tf_base_pose.transform.translation.y,self.tf_base_pose.transform.translation.x)
                # D = math.sqrt(b*b - 4*a*c)
                # x_ld = (-b + math.copysign(D,1)) / (2*a)
                # if(self.tf_base_pose.transform.translation.z < 0):
                #     x_ld = (-b + math.copysign(D,-1)) / (2*a)
                # y_ld = m_goal * x_ld + l_goal

                
        elif (self.path.poses and not self.reach_state ):
            pp_result.data = 0
            yt = self.tf_base_pose.transform.translation.y
            ld_2 = self.ld * self.ld

            steering_angle = max(min( math.atan2(2 * yt * self.L, ld_2), self.steering_max), -self.steering_max)
            # v_accelerate = abs(new_odom.twist.twist.linear.x)+(self.accelerate*(odom_time-self.prev_odom_time)) #V=U+aT
            # v_decelerate = math.sqrt(2*self.decelerate*abs(self.distance(self.path.poses[-1].pose.position, self.tf_map_base.transform.translation)- self.goal_tolerance))  #U = sqrt(2(-a)s)
            # v = max(min(v_decelerate,v_accelerate,self.v_max),self.v_min)
            # print(v_decelerate,v_accelerate,self.v_max,self.v_min)
            if (self.path.poses[self.bb_idx].pose.position.z > 0):
                v = self.path.poses[self.bb_idx].pose.position.z
                pp_angular_vel = float(min( 2*v/ ld_2 * yt, self.w_max))
                pp_linear_vel = float(v)
            elif (self.path.poses[self.bb_idx].pose.position.z < 0):
                v = -self.path.poses[self.bb_idx].pose.position.z
                pp_angular_vel = float(min( -2*v/ ld_2 * yt, self.w_max))
                pp_linear_vel = float(-v)
            else:
                pp_angular_vel = float(0)
                pp_linear_vel = float(0)
        
        else:
            pp_linear_vel = float(0)
            pp_angular_vel = float(0)

            if(pp_result != 3):
                # self.pub_vel.publish(self.pp_vel);
                pp_result.data = 3;
                # self.pub_result.publish(pp_result);
        # self.tf_base_lookahead.header.stamp = self.get_clock().now().to_msg()
        # self.tf_broadcaster.sendTransform(self.tf_base_lookahead)
        # print('pub_pp_vel')
        # self.pub_vel.publish(self.pp_vel)
        # self.prev_odom_time = odom_time
        return(pp_linear_vel,pp_angular_vel)

    def cal_pp_index_and_lookahead(self,tf_map_base,odom):
        self.ld = min((max((odom.twist.twist.linear.x-self.pp_v_min_ld)/(self.pp_v_max_ld-self.pp_v_min_ld),0)),1)*(self.pp_max_ld-self.pp_min_ld)+self.pp_min_ld
        for i in range(self.pp_idx,len(self.path.poses)):
            if self.distance(self.path.poses[i].pose.position,tf_map_base.transform.translation) > self.ld :
                self.pp_idx = i   
                break

    def cal_v_accelerate(self,new_odom):
        if self.path.poses:
            return math.copysign((abs(new_odom.twist.twist.linear.x)+(self.accelerate*(self.odom_time-self.prev_odom_time))),self.path.poses[self.bb_idx].pose.position.z) #V=U+aT
        else:
            return 0
    def cal_v_decelerate(self):
        # return math.copysign((abs(new_odom.twist.twist.linear.x)-(self.decelerate*(self.odom_time-self.prev_odom_time))),new_odom.twist.twist.linear.x) #V=U+aT
        return math.sqrt(2*self.decelerate*abs(((self.path_length-1)-self.bb_idx)*self.path_resolution))  #U = sqrt(2(-a)s)
    
    def angular_vel_to_steer(self,linear_vel,angular_vel):
        return atan2(self.wheel_base * angular_vel, linear_vel) if linear_vel != 0 else 0

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

def main(args=None):
    rclpy.init(args=args)
    bb_purepursuit_ros2 = BB_purepursuit_ros2()
    rclpy.spin(bb_purepursuit_ros2)

    bb_purepursuit_ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()