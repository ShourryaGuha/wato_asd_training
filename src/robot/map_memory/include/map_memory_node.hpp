#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  robot::MapMemoryCore map_memory_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double distance_threshold_m_{5};
  std::chrono::milliseconds update_period_{std::chrono::milliseconds(1000)};

  nav_msgs::msg::OccupancyGrid latest_costmap_;

  bool have_latest_costmap_{false};
  bool first_map_published_{false};

  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};

  double last_update_x_{0.0};
  double last_update_y_{0.0};

  bool should_update_map_{false};

  bool anchor_set_{false};

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  bool movedEnoughSinceLastUpdate() const;
  void markUpdateAnchorToCurrentPose();
  void declareAndGetParameters();
};

#endif  // MAP_MEMORY_NODE_HPP_
