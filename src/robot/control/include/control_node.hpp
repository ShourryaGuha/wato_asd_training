#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    ~ControlNode() noexcept override = default;

  private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;

    double lookahead_distance_{1.0};
    double goal_tolerance_{0.2};
    double linear_speed_{0.5};
    double control_period_ms_{100.0};

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target) const;
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
};

#endif
