#include "map_memory_core.hpp"

#include <cmath>
#include <utility>

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger)
    : logger_(logger) {}

void MapMemoryCore::initializeGlobalMap(const std::string &frame_id,
                                        double resolution,
                                        int width_cells,
                                        int height_cells,
                                        double origin_x,
                                        double origin_y) {
  // Basic sanity
  if (resolution <= 0.0 || width_cells <= 0 || height_cells <= 0) {
    RCLCPP_ERROR(logger_, "Invalid global map configuration (res/size).");
    return;
  }

  global_map_.header.frame_id = frame_id;
  global_map_.info.resolution = resolution;
  global_map_.info.width = static_cast<uint32_t>(width_cells);
  global_map_.info.height = static_cast<uint32_t>(height_cells);

  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  const size_t N = static_cast<size_t>(width_cells) *
                   static_cast<size_t>(height_cells);
  global_map_.data.assign(N, -1); // unknown everywhere

  RCLCPP_INFO(logger_, "Initialized global map: frame='%s', res=%.3f, size=%ux%u, origin=(%.3f, %.3f)",
              global_map_.header.frame_id.c_str(),
              global_map_.info.resolution,
              global_map_.info.width, global_map_.info.height,
              global_map_.info.origin.position.x,
              global_map_.info.origin.position.y);
}

const nav_msgs::msg::OccupancyGrid &MapMemoryCore::globalMap() const {
  return global_map_;
}

void MapMemoryCore::fuseCostmap(const nav_msgs::msg::OccupancyGrid &local_costmap,
                                double robot_x,
                                double robot_y,
                                double robot_yaw) {
  // Ensure we were initialized
  if (global_map_.data.empty()) {
    RCLCPP_WARN(logger_, "Global map not initialized; call initializeGlobalMap() first.");
    return;
  }

  // Fast sin/cos
  const double cy = std::cos(robot_yaw);
  const double sy = std::sin(robot_yaw);

  const bool same_frame =
      (local_costmap.header.frame_id == global_map_.header.frame_id);

  const uint32_t w_local = local_costmap.info.width;
  const uint32_t h_local = local_costmap.info.height;

  if (w_local == 0 || h_local == 0) {
    RCLCPP_WARN(logger_, "Local costmap has zero size; skipping fusion.");
    return;
  }

  // Iterate all local cells
  for (uint32_t i = 0; i < h_local; ++i) {
    for (uint32_t j = 0; j < w_local; ++j) {
      const size_t idx_local = static_cast<size_t>(i) * w_local + j;
      const int8_t v = local_costmap.data[idx_local];

      // Linear fusion rule: skip unknown; known will overwrite
      if (v < 0) {
        continue;
      }

      // Get cell center in the *local* costmap's world frame
      double wx_local = 0.0, wy_local = 0.0;
      localCellCenterToWorld(static_cast<int>(i), static_cast<int>(j),
                             local_costmap, wx_local, wy_local);

      // Transform to *global* world coordinates
      double wx = 0.0, wy = 0.0;
      if (same_frame) {
        wx = wx_local;
        wy = wy_local;
      } else {
        // Treat local coords as robot-local and transform with robot pose
        robotToGlobal(wx_local, wy_local, robot_x, robot_y, robot_yaw, wx, wy);
      }

      // Convert world → global map indices
      int gx = 0, gy = 0;
      if (!worldToGlobalIdx(wx, wy, gx, gy)) {
        // Out of bounds → skip (no auto-grow in this implementation)
        continue;
      }

      const size_t idx_global =
          static_cast<size_t>(gy) * global_map_.info.width + static_cast<size_t>(gx);

      // Overwrite with known value (0..100)
      global_map_.data[idx_global] = v;
    }
  }

  RCLCPP_DEBUG(logger_, "Fused local costmap into global map.");
}

bool MapMemoryCore::worldToGlobalIdx(double wx, double wy, int &gx, int &gy) const {
  const double res = global_map_.info.resolution;
  const double x0 = global_map_.info.origin.position.x;
  const double y0 = global_map_.info.origin.position.y;

  const double fx = (wx - x0) / res;
  const double fy = (wy - y0) / res;

  gx = static_cast<int>(std::floor(fx));
  gy = static_cast<int>(std::floor(fy));

  if (gx < 0 || gy < 0 ||
      gx >= static_cast<int>(global_map_.info.width) ||
      gy >= static_cast<int>(global_map_.info.height)) {
    return false;
  }
  return true;
}

void MapMemoryCore::localCellCenterToWorld(int i, int j,
                                           const nav_msgs::msg::OccupancyGrid &local,
                                           double &wx, double &wy) const {
  const double res = local.info.resolution;
  const double x0 = local.info.origin.position.x;
  const double y0 = local.info.origin.position.y;

  // cell centers
  wx = x0 + (static_cast<double>(j) + 0.5) * res;
  wy = y0 + (static_cast<double>(i) + 0.5) * res;
}

void MapMemoryCore::robotToGlobal(double lx, double ly,
                                  double rx, double ry, double ryaw,
                                  double &gx, double &gy) const {
  const double c = std::cos(ryaw);
  const double s = std::sin(ryaw);
  // rotate + translate
  gx = rx + (c * lx - s * ly);
  gy = ry + (s * lx + c * ly);
}

} // namespace robot
