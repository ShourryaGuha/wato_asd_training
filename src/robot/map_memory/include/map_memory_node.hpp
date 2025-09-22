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
    nav_msgs::msg::OccupancyGrid latest_costmap_; // stores the latest local costmap received from the /costmap topic
    nav_msgs::msg::OccupancyGrid global_map_;    // the global map being built and published to /map

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_; 
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_pub_; 
    rclcpp::TimerBase::SharedPtr                                   timer_; 

    // Robot pose (from /odom/filtered)
    double robot_x_{0.0};
    double robot_y_{0.0};

    // Anchor pose at last fusion
    double last_update_x_{0.0};
    double last_update_y_{0.0};

    bool updated_map_{false};
    bool updated_costmap_{false};


    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();  // checks gates, calls core, publishes /map

    static constexpr uint32_t MAP_PUB_RATE = 1000;
    static constexpr double DIST_THRESH = 1.5;
    static constexpr double RES = 0.1;
    static constexpr int COST_MAP_WIDTH = 300;
    static constexpr int COST_MAP_HEIGHT = 300;
    static constexpr double ORIGIN_MAP_X = 15;
    static constexpr double ORIGIN_MAP_Y = 15;
};

#endif 
