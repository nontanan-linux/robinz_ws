import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, pi, copysign
import sys, traceback


class GenPath(Node):
    def __init__(self):
        super().__init__('gen_path')
        # Create publishers
        self.declare_parameter('agv_name', 'AGV1')
        self.agv_name = self.get_parameter('agv_name').get_parameter_value().string_value

        self.pub_path = self.create_publisher(Path, 'path_segment', 1)
        self.pub_path_display = self.create_publisher(Path, 'path_segment_display', 1)
        self.path_len = self.create_publisher(Int32MultiArray, 'path_len', 1)
        
        # Create subscriber
        self.sub_mode = self.create_subscription(String, self.agv_name+'/mode', self.mode_cb, 1)

        self.csv_path_list = []
        self.path = []
        self.p_len = Int32MultiArray()
        self.csv_folder = '/home/nontanan/ros2_ws/src/py_path_generator/py_path_generator/tb4_sim_map/'

    def mode_cb(self, data):
        self.csv_path_list = []
        self.path = []
        self.p_len.data = []
        
        if 'path' in data.data or 'bb' in data.data:
            nodes = data.data.split('|')[1:]
            for i in range(len(nodes) - 1):
                self.csv_path_list.append(self.csv_folder + nodes[i] + '-' + nodes[i + 1] + '.csv')
            self.get_logger().info(f"CSV Path List: {self.csv_path_list}")

            try:
                for path_file in self.csv_path_list:
                    with open(path_file) as file:
                        self.path += [([float(x) for x in line.split(",")]) for line in file.readlines()]               
                        self.p_len.data = list(self.p_len.data)+[len(self.path)]
            except Exception as e:
                self.get_logger().error(f"File {path_file} error: {e}")
                self.robot_mode = ''

            ros_path = Path()
            ros_path.header.frame_id = "map"

            # Creating poses for each point in the path
            for idx, p in enumerate(self.path):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                try:
                    pose.pose.position.z = p[2]
                except IndexError:
                    pose.pose.position.z = 1.0

                try:
                    if (self.path[idx + 1][0] - p[0]) != 0:
                        q = quaternion_from_euler(0, 0, atan2(self.path[idx + 1][1] - p[1], self.path[idx + 1][0] - p[0]))
                    else:
                        q = quaternion_from_euler(0, 0, copysign(pi / 2, (self.path[idx + 1][1] - p[1])))
                except:
                    q = [0, 0, 0, 1]  # Default quaternion in case of error
                
                pose.pose.orientation.x = float(q[0])
                pose.pose.orientation.y = float(q[1])
                pose.pose.orientation.z = float(q[2])
                pose.pose.orientation.w = float(q[3])
                ros_path.poses.append(pose)

            self.pub_path.publish(ros_path)
            self.p_len.data = self.cal_len(self.p_len.data)
            self.path_len.publish(self.p_len)

            ros_path_display = Path()
            ros_path_display.header.frame_id = "map"
            for idx, p in enumerate(self.path):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                try:
                    pose.pose.position.z = 1.0
                except IndexError:
                    pose.pose.position.z = 1.0

                try:
                    if (self.path[idx + 1][0] - p[0]) != 0:
                        q = quaternion_from_euler(0, 0, atan2(self.path[idx + 1][1] - p[1], self.path[idx + 1][0] - p[0]))
                    else:
                        q = quaternion_from_euler(0, 0, copysign(pi / 2, (self.path[idx + 1][1] - p[1])))
                except:
                    q = [0.0, 0.0, 0.0, 1.0]  # Default quaternion in case of error
                
                pose.pose.orientation.x = float(q[0])
                pose.pose.orientation.y = float(q[1])
                pose.pose.orientation.z = float(q[2])
                pose.pose.orientation.w = float(q[3])
                ros_path_display.poses.append(pose)

            self.pub_path_display.publish(ros_path_display)
            self.robot_mode = ''

        elif data.data in ['pause', 'pauseobstacle', 'continue', 'continueobstacle']:
            pass
        else:
            ros_path = Path()
            ros_path.header.frame_id = "map"
            self.pub_path.publish(ros_path)
            self.pub_path_display.publish(ros_path)

    def cal_len(self, data_list):
        new_len = []
        for idx in range(len(data_list)):
            if idx > 0:
                new_len += [data_list[idx] - data_list[idx - 1]]
            else:
                new_len = [data_list[idx]]
        return new_len


def main(args=None):
    rclpy.init(args=args)
    path_generator = GenPath()

    # Spin until shutdown
    rclpy.spin(path_generator)

    # Cleanup
    path_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
