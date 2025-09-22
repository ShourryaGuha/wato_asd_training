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
  robot::MapMemoryCore map_memory_; // this is an instance of the map_memory_core class

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_; // class of type <nav_msgs: ... > that listens for messages of type <nav_msgs: msg::OccupancyGrid>, subscribes to costmap topic (where robot's local costmap is published)
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_; // class of type <nav_msgs: ... > that listens for messages of type <nav_msgs: msg::Odometry>, subscribes to odom/filtered topic (where robot's filtered odometry is published): gives the robots current position and orientation
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_pub_; // class of type <nav_msgs: ... > that publishes messages of type <nav_msgs: msg::OccupancyGrid>, publishes to map topic (where fused global map is published): publishes the fused global map to the /map topic
  rclcpp::TimerBase::SharedPtr                                   timer_; // timer to periodically check if we should fuse and publish, calls timerCallback() at fixed intervals (defined by update_period_)

  // -------- Parameters --------
  // Distance the robot must move before fusing (meters), e.g., 1.5
  double distance_threshold_m_{1.5};
  // Minimum period between fusions e.g. 1000 ms
  std::chrono::milliseconds update_period_{std::chrono::milliseconds(1000)}; // using the chrono library because it provides a type-safe way to represent time durations and points in time

  // Latest local costmap snapshot (from /costmap)
  nav_msgs::msg::OccupancyGrid latest_costmap_; // stores the latest local costmap received from the /costmap topic
  
  bool have_latest_costmap_{false}; // flag to indicate if we have received at least one costmap
  bool first_map_published_{false}; // flag to indicate if we have published the initial map

  // Robot pose (from /odom/filtered)
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};   // optional, if needed by core

  // Anchor pose at last fusion
  double last_update_x_{0.0};
  double last_update_y_{0.0};

  // Gate to avoid fusing too often
  bool should_update_map_{false};

  bool anchor_set_{false}; // flag to indicate if the anchor pose has been set at least once

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();  // checks gates, calls core, publishes /map

  // -------- Helpers --------
  bool movedEnoughSinceLastUpdate() const;
  void markUpdateAnchorToCurrentPose();
  void declareAndGetParameters();  // declare/get distance_threshold_m_, update_period_
};

#endif  // MAP_MEMORY_NODE_HPP_
