#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
using namespace std;

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

 

void MapMemoryCore::fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw){
  if(!global_map_initialized_){
    initializeGlobalMap(costmap, robot_x, robot_y, robot_yaw);
    global_map_initialized_ = true;
    RCLCPP_INFO(logger_, "Global map initialized.");
    return;
  }
  for (size_t y = 0; y < costmap.info.height; ++y){
    for (size_t x = 0; x < costmap.info.width; ++x){
      int costmap_index = x + y * costmap.info.width;
      int8_t costmap_value = costmap.data[costmap_index];
      if (costmap_value == -1) continue; // unknown in local costmap, skip

      double world_x = costmap.info.origin.position.x + (x + 0.5) * costmap.info.resolution;
      double world_y = costmap.info.origin.position.y + (y + 0.5) * costmap.info.resolution;

      double rotated_x = cos(robot_yaw) * (world_x) - sin(robot_yaw) * (world_y);
      double rotated_y = sin(robot_yaw) * (world_x) + cos(robot_yaw) * (world_y);

      double global_x = rotated_x + robot_x;
      double global_y = rotated_y + robot_y;

      int global_x_index = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int global_y_index = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

      if (global_x_index < 0 || global_x_index >= static_cast<int>(global_map_.info.width) ||
          global_y_index < 0 || global_y_index >= static_cast<int>(global_map_.info.height)) {
        continue; // out of bounds
      }

      int global_index = global_x_index + global_y_index * global_map_.info.width;
      int8_t& global_value = global_map_.data[global_index];

      if (costmap_value == 100) {
        global_value = 100; // occupied
      } else if (costmap_value == 0 && global_value != 100) {
        global_value = 0; // free, only update if not already occupied
      }
    }
  }
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

void MapMemoryCore::initializeGlobalMap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw) {
  global_map_ = costmap; // start with the local costmap as a base

  double cos_yaw = cos(robot_yaw);
  double sin_yaw = sin(robot_yaw);

  double origin_x = robot_x - (cos_yaw * (costmap.info.width * costmap.info.resolution) / 2.0) + (sin_yaw * (costmap.info.height * costmap.info.resolution) / 2.0);
  double origin_y = robot_y - (sin_yaw * (costmap.info.width * costmap.info.resolution) / 2.0) - (cos_yaw * (costmap.info.height * costmap.info.resolution) / 2.0);

  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, robot_yaw);
  global_map_.info.origin.orientation.x = q.x();
  global_map_.info.origin.orientation.y = q.y();
  global_map_.info.origin.orientation.z = q.z();
  global_map_.info.origin.orientation.w = q.w();

  for (auto& cell : global_map_.data) {
    cell = -1; // unknown
  }
}

}