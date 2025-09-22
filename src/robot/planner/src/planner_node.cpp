#include "planner_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

PlannerNode::PlannerNode() : Node("planner"), planner_{this->get_logger()} {
  // declare parameters
  plan_period_ms_ = this->declare_parameter<double>("plan_period_ms", 500.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  replan_timeout_s_ = this->declare_parameter<double>("replan_timeout_s", 5.0);
  occ_threshold_ = this->declare_parameter<int>("occ_threshold", 50);
  unknown_is_free_ = this->declare_parameter<bool>("unknown_is_free", true);
  use_8_connected_ = this->declare_parameter<bool>("use_8_connected", true);

  // subscriptions
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::SensorDataQoS(),
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", rclcpp::QoS(10),
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", rclcpp::QoS(50),
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  // publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(10));

  // timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(plan_period_ms_)),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  planner_.setMap(current_map_, occ_threshold_, unknown_is_free_, use_8_connected_);
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath(); // replan on new map
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  have_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)",
              goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}
bool PlannerNode::goalReached() const
{
  const double dx = goal_.point.x - robot_pose_.position.x;
  const double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx*dx + dy*dy) < goal_tolerance_;
}
void PlannerNode::planPath()
{
  if (!have_goal_ || current_map_.data.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Cannot plan: missing goal or map");
    return;
  }

  const double sx = robot_pose_.position.x;
  const double sy = robot_pose_.position.y;
  const double gx = goal_.point.x;
  const double gy = goal_.point.y;

  auto poses = planner_.plan(sx, sy, gx, gy);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->now();
  path.poses = std::move(poses);

  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "A* failed to find a path");
  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Planned path with %zu poses", path.poses.size());
  }

  path_pub_->publish(path);
  last_plan_time_ = this->now();
}

void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_GOAL) return;

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    have_goal_ = false;
    return;
  }

  // replan on timeout
  const double dt = (this->now() - last_plan_time_).seconds();
  if (dt >= replan_timeout_s_) {
    RCLCPP_INFO(this->get_logger(), "Replanning due to timeout (%.1fs)...", dt);
    planPath();
  }
}

// main function
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
