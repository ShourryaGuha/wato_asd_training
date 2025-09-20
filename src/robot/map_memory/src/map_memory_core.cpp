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
  
  int occupied_cells_fused = 0;
  int free_cells_fused = 0;
  int out_of_bounds_cells = 0;
  int unknown_cells_skipped = 0;
  
  RCLCPP_INFO(logger_, "Fusing costmap: robot at (%.2f, %.2f), costmap %dx%d, origin (%.2f, %.2f)", 
             robot_x, robot_y, costmap.info.width, costmap.info.height,
             costmap.info.origin.position.x, costmap.info.origin.position.y);
  
  for (size_t y = 0; y < costmap.info.height; ++y){
    for (size_t x = 0; x < costmap.info.width; ++x){
      int costmap_index = x + y * costmap.info.width;
      int8_t costmap_value = costmap.data[costmap_index];
      if (costmap_value == -1) {
        unknown_cells_skipped++;
        continue; // unknown in local costmap, skip
      }

      // Convert costmap cell coordinates to world coordinates
      double world_x = costmap.info.origin.position.x + (x + 0.5) * costmap.info.resolution;
      double world_y = costmap.info.origin.position.y + (y + 0.5) * costmap.info.resolution;

      // Since both costmap and global map are in "map" frame, no rotation needed
      // Just use the world coordinates directly
      double global_x = world_x;
      double global_y = world_y;

      int global_x_index = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int global_y_index = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

      if (global_x_index < 0 || global_x_index >= static_cast<int>(global_map_.info.width) ||
          global_y_index < 0 || global_y_index >= static_cast<int>(global_map_.info.height)) {
        out_of_bounds_cells++;
        continue; // out of bounds
      }

      int global_index = global_x_index + global_y_index * global_map_.info.width;
      int8_t& global_value = global_map_.data[global_index];

      // Handle all costmap values (0-100) properly
      if (costmap_value >= 0) {  // Any known value (0-100)
        // Use maximum value fusion: keep the higher cost between existing and new
        if (global_value == -1 || costmap_value > global_value) {
          global_value = costmap_value;
          if (costmap_value == 100) occupied_cells_fused++;
          else if (costmap_value == 0) free_cells_fused++;
        }
      }
      // If costmap_value is -1 (unknown), we already skip it above
    }
  }
  
  RCLCPP_INFO(logger_, "Fusion complete: %d occupied, %d free, %d out-of-bounds, %d unknown skipped", 
             occupied_cells_fused, free_cells_fused, out_of_bounds_cells, unknown_cells_skipped);
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getGlobalMap() const {
  return global_map_;
}

void MapMemoryCore::initializeGlobalMap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw) {
  // Create a fixed-size global map that covers the entire environment
  global_map_.info.resolution = costmap.info.resolution; // Same resolution as costmap (0.1m)
  
  // Fixed size for 30m x 30m environment at 0.1m resolution = 300x300 cells
  global_map_.info.width = 300;
  global_map_.info.height = 300;
  
  // Set origin so that the 30m x 30m map is centered around (0,0)
  // This means the map covers from -15m to +15m in both x and y
  global_map_.info.origin.position.x = -15.0;
  global_map_.info.origin.position.y = -15.0;
  global_map_.info.origin.position.z = 0.0;
  
  // Identity orientation - "map" frame
  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  // Initialize all cells to unknown (-1)
  global_map_.data.resize(global_map_.info.width * global_map_.info.height);
  for (auto& cell : global_map_.data) {
    cell = -1; // unknown
  }
  
  RCLCPP_INFO(logger_, "Global map initialized: %dx%d (30m x 30m) at origin (%.1f, %.1f)", 
             global_map_.info.width, global_map_.info.height,
             global_map_.info.origin.position.x, global_map_.info.origin.position.y);
}

}