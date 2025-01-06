import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

class DWA(Node):
    def __init__(self):
        super().__init__('dwa_node')

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for odometry data
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Subscriber for goal position
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)

        # Current state and goal
        self.current_position = (0.0, 0.0)
        self.current_velocity = (0.0, 0.0)
        self.goal_position = (0.0, 0.0)

        # DWA parameters
        self.max_speed = 0.5  # Maximum speed
        self.max_yaw_rate = 1.0  # Maximum yaw rate
        self.v_resolution = 0.1  # Velocity resolution
        self.yaw_rate_resolution = 0.1  # Yaw rate resolution
        self.dt = 0.1  # Time increment

        self.get_logger().info("Dynamic Window Approach Node Initialized")

    def odom_callback(self, msg: Odometry):
        # Update the current position and velocity from odometry data
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def goal_callback(self, msg: PoseStamped):
        # Update the goal position
        self.goal_position = (msg.pose.position.x, msg.pose.position.y)

        # Compute the best velocity command
        velocity_command = self.compute_velocity_command()

        # Publish the velocity command
        self.velocity_publisher.publish(velocity_command)

    def compute_velocity_command(self):
        best_velocity = Twist()
        best_cost = float('inf')

        # Iterate through possible velocities
        for v in np.arange(0, self.max_speed, self.v_resolution):
            for w in np.arange(-self.max_yaw_rate, self.max_yaw_rate, self.yaw_rate_resolution):
                # Simulate the next state
                x_next, y_next, theta_next = self.simulate_motion(v, w)

                # Compute the cost for the current velocity
                cost = self.compute_cost((x_next, y_next), theta_next)

                if cost < best_cost:
                    best_cost = cost
                    best_velocity.linear.x = v
                    best_velocity.angular.z = w

        return best_velocity

    def simulate_motion(self, v, w):
        # Simulate the next position and orientation
        x, y = self.current_position
        theta = np.arctan2(self.current_velocity[1], self.current_velocity[0])  # Current heading

        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + w * self.dt

        return x_next, y_next, theta_next

    def compute_cost(self, position, theta):
        # Compute cost based on distance to goal and orientation
        goal_x, goal_y = self.goal_position
        distance_to_goal = np.sqrt((goal_x - position[0])**2 + (goal_y - position[1])**2)

        # Orientation cost
        heading = np.arctan2(goal_y - position[1], goal_x - position[0])
        orientation_cost = np.abs(theta - heading)

        # Total cost (you can weight these terms as needed)
        total_cost = distance_to_goal + orientation_cost
        return total_cost

def main(args=None):
    rclpy.init(args=args)
    dwa_node = DWA()
    rclpy.spin(dwa_node)

    dwa_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
