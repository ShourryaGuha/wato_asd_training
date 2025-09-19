#include "costmap_core.hpp"

namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger &logger) : logger_(logger) {}

    void CostmapCore::convertToGrid(float range, float angle, float& x_grid, float& y_grid) {
        x_grid = range * cos(angle);
        y_grid = range * sin(angle);
    }

    void CostmapCore::markObstacle(std::vector<std::vector<int>>& costmap_2D, int x_grid, int y_grid) {
        if (x_grid >= 0 && x_grid < costmap_2D[0].size() && y_grid >= 0 && y_grid < costmap_2D.size()) {
            costmap_2D[y_grid][x_grid] = 100; // Mark as occupied
            // RCLCPP_INFO(logger_, "Marked obstacle at (%d, %d)", x_grid, y_grid);
        } 
    }

}