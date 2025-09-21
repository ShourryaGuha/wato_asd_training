#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <mutex>
#include <string>

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  // Core fusion engine
  robot::MapMemoryCore map_memory_;

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double update_distance_threshold_{1.5};
  std::string map_frame_id_{"map"};
  double resolution_{0.10};
  int width_cells_{800};
  int height_cells_{800};
  double origin_x_{-40.0};
  double origin_y_{-40.0};

  // State (protected by mutex)
  std::mutex mtx_;
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_costmap_{false};
  bool should_update_{false};
  double last_update_x_{0.0};
  double last_update_y_{0.0};

  // Helpers
  void declareAndGetParams();
  void initGlobalMapViaCore();

  void onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  void timerCallback();

  static double yawFromQuat(double x, double y, double z, double w);
};

#endif  // MAP_MEMORY_NODE_HPP_
