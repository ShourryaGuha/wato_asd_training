#include "control_node.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

ControlNode::ControlNode()
: Node("control")
{
  const auto path_topic = this->declare_parameter<std::string>("path_topic", "/path");
  const auto odom_topic = this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  const auto cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", lookahead_distance_);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", goal_tolerance_);
  linear_speed_ = this->declare_parameter<double>("linear_speed", linear_speed_);
  control_period_ms_ = this->declare_parameter<double>("control_period_ms", control_period_ms_);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic, 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(control_period_ms_)),
    std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
        "Control node ready: path=%s odom=%s cmd_vel=%s lookahead=%.2f speed=%.2f",
        path_topic.c_str(), odom_topic.c_str(), cmd_vel_topic.c_str(),
        lookahead_distance_, linear_speed_);
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() const
{
  if (!current_path_ || !current_odom_ || current_path_->poses.empty()) {
    return std::nullopt;
  }

  const auto& robot_position = current_odom_->pose.pose.position;
  for (const auto& pose : current_path_->poses) {
    if (computeDistance(pose.pose.position, robot_position) >= lookahead_distance_) {
      return pose;
    }
  }

  return current_path_->poses.back();
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point& a,
                                    const geometry_msgs::msg::Point& b) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& quat) const
{
  const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped& target) const
{
  geometry_msgs::msg::Twist cmd_vel;

  if (!current_odom_) {
    return cmd_vel;
  }

  const auto& robot_position = current_odom_->pose.pose.position;
  const auto& robot_orientation = current_odom_->pose.pose.orientation;

  if (computeDistance(target.pose.position, robot_position) <= goal_tolerance_) {
    return cmd_vel;
  }

  const double robot_yaw = extractYaw(robot_orientation);
  const double dx = target.pose.position.x - robot_position.x;
  const double dy = target.pose.position.y - robot_position.y;

  const double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  if (lookahead_distance_ <= 1e-6) {
    return cmd_vel;
  }

  const double curvature = (2.0 * local_y) / (lookahead_distance_ * lookahead_distance_);
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = linear_speed_ * curvature;

  if (local_x < 0.0) {
    cmd_vel.linear.x = 0.0;
  }

  return cmd_vel;
}

void ControlNode::controlLoop()
{
  if (!current_path_ || !current_odom_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  if (current_path_->poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  const auto target = findLookaheadPoint();
  if (!target) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  const auto& robot_position = current_odom_->pose.pose.position;
  if (computeDistance(target->pose.position, robot_position) <= goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  auto cmd = computeVelocity(*target);
  cmd_vel_pub_->publish(cmd);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "cmd_vel: linear=%.3f angular=%.3f target=(%.2f, %.2f)",
                       cmd.linear.x, cmd.angular.z,
                       target->pose.position.x, target->pose.position.y);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
