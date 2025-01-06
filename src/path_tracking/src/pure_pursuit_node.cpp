// pure_pursuit_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "pure_pursuit.hpp"

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode() : Node("pure_pursuit_node"), pure_pursuit_(1.0, 1.0), linear_velocity_(1.0)
    {
        // Parameters
        this->declare_parameter("look_ahead_distance", 1.0);
        this->get_parameter("look_ahead_distance", look_ahead_distance_);

        this->declare_parameter("wheelbase", 1.0);
        this->get_parameter("wheelbase", wheelbase_);

        this->declare_parameter("linear_velocity", 1.0);
        this->get_parameter("linear_velocity", linear_velocity_);

        // Initialize Pure Pursuit
        pure_pursuit_ = PurePursuit(look_ahead_distance_, wheelbase_);

        // Subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        if (!path_.poses.empty())
        {
            computeSteeringAngle();
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = *msg;
    }

    void computeSteeringAngle()
    {
        geometry_msgs::msg::PoseStamped lookahead_point;
        if (!pure_pursuit_.findLookaheadPoint(path_, current_pose_, lookahead_point))
        {
            RCLCPP_WARN(this->get_logger(), "No valid look-ahead point found.");
            return;
        }

        double steering_angle = pure_pursuit_.computeSteeringAngle(current_pose_, lookahead_point);

        // Create and publish Twist message
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity_;
        cmd_vel.angular.z = steering_angle;
        cmd_vel_pub_->publish(cmd_vel);
    }

    // Node variables
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    PurePursuit pure_pursuit_;
    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::Path path_;
    double look_ahead_distance_;
    double wheelbase_;
    double linear_velocity_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
