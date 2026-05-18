#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a,
                                     const geometry_msgs::msg::Point& b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) const
{
  const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    const nav_msgs::msg::Odometry& odom,
    double lookahead_distance) const
{
  if (path.poses.empty()) {
    return std::nullopt;
  }

  const auto& robot_position = odom.pose.pose.position;
  for (const auto& pose : path.poses) {
    if (computeDistance(pose.pose.position, robot_position) >= lookahead_distance) {
      return pose;
    }
  }

  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const nav_msgs::msg::Path& path,
    const nav_msgs::msg::Odometry& odom,
    const geometry_msgs::msg::PoseStamped& target,
    double lookahead_distance,
    double linear_speed,
    double goal_tolerance) const
{
  geometry_msgs::msg::Twist twist;

  const auto& robot_position = odom.pose.pose.position;
  const auto& robot_orientation = odom.pose.pose.orientation;

  if (path.poses.empty() || computeDistance(target.pose.position, robot_position) <= goal_tolerance) {
    return twist;
  }

  const double robot_yaw = extractYaw(robot_orientation);
  const double dx = target.pose.position.x - robot_position.x;
  const double dy = target.pose.position.y - robot_position.y;

  const double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  if (lookahead_distance <= 1e-6) {
    RCLCPP_WARN(logger_, "Lookahead distance is too small, stopping robot");
    return twist;
  }

  const double curvature = (2.0 * local_y) / (lookahead_distance * lookahead_distance);
  twist.linear.x = linear_speed;
  twist.angular.z = linear_speed * curvature;

  if (local_x < 0.0) {
    twist.linear.x = 0.0;
  }

  return twist;
}

}  
