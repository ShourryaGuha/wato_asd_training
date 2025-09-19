#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <string>

namespace robot {

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger &logger);

  // One-time setup for the global map (called by the node at startup)
  void initializeGlobalMap(const std::string &frame_id,
                           double resolution,
                           int width_cells,
                           int height_cells,
                           double origin_x,
                           double origin_y);

  // Fuse a local costmap snapshot into the global map.
  // robot_x/robot_y/robot_yaw are the robot pose in the *global* frame
  // at (or close to) the time the local costmap was produced.
  void fuseCostmap(const nav_msgs::msg::OccupancyGrid &local_costmap,
                   double robot_x,
                   double robot_y,
                   double robot_yaw);

  // Accessor for publishing (the node sets header.stamp and publishes)
  const nav_msgs::msg::OccupancyGrid &globalMap() const;

private:
  rclcpp::Logger logger_;
  nav_msgs::msg::OccupancyGrid global_map_;

  // --- helpers ---
  // Convert world (global) coordinates to integer grid indices in global_map_.
  // Returns false if out-of-bounds.
  bool worldToGlobalIdx(double wx, double wy, int &gx, int &gy) const;

  // Compute the *world* coordinates of the center of a local cell (i,j)
  // in the *local costmap's* frame.
  void localCellCenterToWorld(int i, int j,
                              const nav_msgs::msg::OccupancyGrid &local,
                              double &wx, double &wy) const;

  // Transform a point from robot-local frame to global frame using robot pose.
  // (Rotate by yaw, then translate by (rx, ry).)
  void robotToGlobal(double lx, double ly,
                     double rx, double ry, double ryaw,
                     double &gx, double &gy) const;
};

} // namespace robot

#endif // MAP_MEMORY_CORE_HPP_
