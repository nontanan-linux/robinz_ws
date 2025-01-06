// pure_pursuit.cpp

#include "pure_pursuit.hpp"

PurePursuit::PurePursuit(double look_ahead_distance, double wheelbase)
: look_ahead_distance_(look_ahead_distance), wheelbase_(wheelbase)
{
}

bool PurePursuit::findLookaheadPoint(const nav_msgs::msg::Path &path, const geometry_msgs::msg::Pose &current_pose, geometry_msgs::msg::PoseStamped &lookahead_point)
{
    for (const auto &pose : path.poses)
    {
        double dx = pose.pose.position.x - current_pose.position.x;
        double dy = pose.pose.position.y - current_pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance >= look_ahead_distance_)
        {
            lookahead_point = pose;
            return true;
        }
    }
    return false;
}

double PurePursuit::computeSteeringAngle(const geometry_msgs::msg::Pose &current_pose, const geometry_msgs::msg::PoseStamped &lookahead_point)
{
    double dx = lookahead_point.pose.position.x - current_pose.position.x;
    double dy = lookahead_point.pose.position.y - current_pose.position.y;
    double alpha = std::atan2(dy, dx) - tf2::getYaw(current_pose.orientation);

    // Pure pursuit steering angle formula
    return std::atan2(2.0 * wheelbase_ * std::sin(alpha), look_ahead_distance_);
}
