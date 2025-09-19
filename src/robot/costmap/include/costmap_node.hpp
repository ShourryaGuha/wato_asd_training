#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void CostmapNode::initializeCostmap(int costmap_width, int costmap_height);
 
  private:
    robot::CostmapCore costmap_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // costmap dimensions
    const int costmap_width_{30};
    const int costmap_height_{30};
    float costmap_resolution_{0.1}; // meters per cell
    double origin_x_{0.0};
    double origin_y_{0.0};
    std::vector<std::vector<int>> costmap_2D;
};
 
#endif