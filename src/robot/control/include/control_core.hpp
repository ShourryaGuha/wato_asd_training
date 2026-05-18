#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path& path,
        const nav_msgs::msg::Odometry& odom,
        double lookahead_distance) const;

    geometry_msgs::msg::Twist computeVelocity(
        const nav_msgs::msg::Path& path,
        const nav_msgs::msg::Odometry& odom,
        const geometry_msgs::msg::PoseStamped& target,
        double lookahead_distance,
        double linear_speed,
        double goal_tolerance) const;

    double computeDistance(const geometry_msgs::msg::Point& a,
                           const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
  
  private:
    rclcpp::Logger logger_;
};

} 

#endif 
