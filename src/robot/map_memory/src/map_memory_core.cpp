#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
using namespace std;

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

 

void MapMemoryCore::fuseCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                double robot_x, double robot_y, double robot_yaw) {
  if (!global_map_initialized_) {
    initializeGlobalMap(costmap, robot_x, robot_y, robot_yaw);
    global_map_initialized_ = true;
    RCLCPP_INFO(logger_, "Global map initialized.");
    return;
  }

  const double res = costmap.info.resolution;
  const double cox = costmap.info.origin.position.x;
  const double coy = costmap.info.origin.position.y;

  const double gx0 = global_map_.info.origin.position.x;
  const double gy0 = global_map_.info.origin.position.y;
  const double gres = global_map_.info.resolution;  // should match res

  const int gW = static_cast<int>(global_map_.info.width);
  const int gH = static_cast<int>(global_map_.info.height);

  const double cY = std::cos(robot_yaw);
  const double sY = std::sin(robot_yaw);

  int occupied_cells_fused = 0;
  int free_cells_fused = 0;
  int out_of_bounds_cells = 0;
  int unknown_cells_skipped = 0;

  RCLCPP_INFO(logger_,
              "Fusing costmap: robot(%.2f, %.2f, yaw=%.2f rad), size=%dx%d, res=%.3f, origin(%.2f, %.2f)",
              robot_x, robot_y, robot_yaw,
              costmap.info.width, costmap.info.height, res, cox, coy);

  for (size_t y = 0; y < costmap.info.height; ++y) {
    for (size_t x = 0; x < costmap.info.width; ++x) {
      const int cidx = static_cast<int>(x + y * costmap.info.width);
      const int8_t costmap_value = costmap.data[cidx];

      // Skip unknown local cells per spec
      if (costmap_value < 0) {
        ++unknown_cells_skipped;
        continue;
      }

      // Cell center in local costmap frame
      const double cx = cox + (static_cast<double>(x) + 0.5) * res;
      const double cy = coy + (static_cast<double>(y) + 0.5) * res;

      // Transform into global "map" frame using robot pose
      const double wx = robot_x + (cx * cY - cy * sY);
      const double wy = robot_y + (cx * sY + cy * cY);

      // Index into global map
      const int gi = static_cast<int>(std::floor((wx - gx0) / gres));
      const int gj = static_cast<int>(std::floor((wy - gy0) / gres));

      if (gi < 0 || gi >= gW || gj < 0 || gj >= gH) {
        ++out_of_bounds_cells;
        continue;
      }

      const int gidx = gi + gj * gW;
      int8_t& gval = global_map_.data[gidx];

      // Linear fusion per assignment: overwrite when local is known (0..100)
      gval = costmap_value;

      if (costmap_value == 100)      ++occupied_cells_fused;
      else if (costmap_value == 0)   ++free_cells_fused;
    }
  }

  RCLCPP_INFO(logger_,
              "Fusion complete: occupied=%d, free=%d, oob=%d, unknown_skipped=%d",
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