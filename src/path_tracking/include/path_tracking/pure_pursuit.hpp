// pure_pursuit.hpp

#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class PurePursuit
{
public:
    PurePursuit(double look_ahead_distance, double wheelbase);

    bool findLookaheadPoint(const nav_msgs::msg::Path &path, const geometry_msgs::msg::Pose &current_pose, geometry_msgs::msg::PoseStamped &lookahead_point);
    double computeSteeringAngle(const geometry_msgs::msg::Pose &current_pose, const geometry_msgs::msg::PoseStamped &lookahead_point);

private:
    double look_ahead_distance_;
    double wheelbase_;
};

#endif // PURE_PURSUIT_HPP
