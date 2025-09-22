#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                  double robot_x, double robot_y, double robot_yaw); // fuse the local costmap into the global map
    const nav_msgs::msg::OccupancyGrid& getGlobalMap() const; // return the current global map
  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid global_map_; // the fused global map, the persistant map
    bool global_map_initialized_{false}; // flag to indicate if the global map has been initialized
    void initializeGlobalMap(const nav_msgs::msg::OccupancyGrid& costmap,
                             double robot_x, double robot_y, double robot_yaw); // initialize the global map based on the first received costmap
};

}  

#endif  