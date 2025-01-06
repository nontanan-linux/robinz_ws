#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import ast

from nav_msgs.msg import Path
from std_msgs.msg import Header,String
from geometry_msgs.msg import PoseStamped,Pose
import math
import numpy as np


def quaternion_from_euler(roll, pitch, yaw):
    ci = math.cos(roll*0.5)
    si = math.sin(roll*0.5)
    cj = math.cos(pitch*0.5)
    sj = math.sin(pitch*0.5)
    ck = math.cos(yaw*0.5)
    sk = math.sin(yaw*0.5)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class route_generator(Node):

    def __init__(self):
        super().__init__('route_generator')

        self.path_name = ''
        self.robot_mode = ''
        self.path = []

        self.csv_path = '/home/gs/ros2_ws/src/py_path_generator/py_path_generator/path_pursuit/default_path.csv'
        self.route_path = []
        self.route = []
        
        self.sub_mode = self.create_subscription(String, 'mode',self.mode_cb,10)
        self.sub_route = self.create_subscription(String, 'route',self.route_cb,10)

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.pub_path = self.create_publisher(Path, 'path_segment', 10)
        # self.main_loop_timer = 0.1  # seconds
        # self.timer = self.create_timer(self.main_loop_timer, self.main_loop_callback)
        

    def main_loop_callback(self):
        if 'path' in self.robot_mode and self.path == [] and self.path_name != '':
            with open(self.csv_path) as file:
                self.path = [([float(x) for x in line.split(",")]) for line in file.readlines()]
            ros_path = Path()
            ros_path.header = Header()

            ros_path.header.frame_id = "map"
            #ros_path.poses = []
            for idx,p in enumerate(self.path):
                pose = PoseStamped()
                pose.header = Header()
                pose.header.frame_id = "map"
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                pose.pose.position.z = p[2]
                try:
                    pose.pose.position.z = p[2]
                except:
                    pose.pose.position.z = 1
                try:
                    if ((self.path[idx+1][0]-p[0]) != 0):
                        q = quaternion_from_euler(0,0,math.atan2((self.path[idx+1][1]-p[1]),(self.path[idx+1][0]-p[0])))
                    else:
                        q = quaternion_from_euler(0,0,copysign(pi/2,(self.path[idx+1][1]-p[1])))
                except:
                    pass
                pose.pose.orientation.x = (q[0])
                pose.pose.orientation.y = (q[1])
                pose.pose.orientation.z = (q[2])
                pose.pose.orientation.w = (q[3])
                ros_path.poses.append(pose)

            self.pub_path.publish(ros_path)
            print('gen path test96')

    def mode_cb(self,data):
        self.robot_mode = data.data
        
        if ('path' in data.data):
            self.path = []
            self.path_name = data.data.split('|')[1] + '_' + data.data.split('|')[2]
            self.csv_path = '/home/gs/ros2_ws/src/py_path_generator/py_path_generator/path_pursuit/'+self.path_name+'.csv'
        else:
            pass

    def route_cb(self,data):
        self.route = (ast.literal_eval(data.data))
        self.route_path = []
        self.path = []
        for idx,val in enumerate(self.route):
            if idx == len(self.route)-1:
                break
            else:
                route_path = '/home/gs/ros2_ws/src/py_path_generator/py_path_generator/golf_dummy_path/'+self.route[idx][0]+"_"+self.route[idx+1][0]+'.csv'
                self.route_path.append(route_path)
        print(self.route_path)
        for r in self.route_path:
            with open(r) as file:
                path = [([float(x) for x in line.split(",")]) for line in file.readlines()]
                self.path.extend(path)
        print(self.path)
        ros_path = Path()
        ros_path.header = Header()

        ros_path.header.frame_id = "map"
        #ros_path.poses = []
        for idx,p in enumerate(self.path):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.frame_id = "map"
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            try:
                pose.pose.position.z = p[2]
            except:
                pose.pose.position.z = 1
            try:
                if ((self.path[idx+1][0]-p[0]) != 0):
                    q = quaternion_from_euler(0,0,math.atan2((self.path[idx+1][1]-p[1]),(self.path[idx+1][0]-p[0])))
                else:
                    q = quaternion_from_euler(0,0,math.copysign(math.pi/2,(self.path[idx+1][1]-p[1])))
            except Exception as e:
                print(e)
            pose.pose.orientation.x = (q[0])
            pose.pose.orientation.y = (q[1])
            pose.pose.orientation.z = (q[2])
            pose.pose.orientation.w = (q[3])
            ros_path.poses.append(pose)

        self.pub_path.publish(ros_path)


def main(args=None):
    rclpy.init(args=args)

    r = route_generator()

    rclpy.spin(r)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()