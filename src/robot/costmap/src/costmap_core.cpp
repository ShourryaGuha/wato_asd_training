#include "costmap_core.hpp"

namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger &logger) : logger_(logger) {}

    void CostmapCore::initializeCostmap(std::vector<std::vector<int>>& costmap_2D, int costmap_width, int costmap_height)
    {
        RCLCPP_INFO(logger_, "Costmap initialization starts with dimensions %dx%d", costmap_width, costmap_height);

        // Resize the 2D vector to the specified dimensions
        costmap_2D.resize(costmap_height);
        for (int i = 0; i < costmap_height; i++) {
            costmap_2D[i].resize(costmap_width);
        }

        // Initialize all cells to 0 (free space)
        for(int i = 0; i < costmap_height; i++)
        {
            for(int j = 0; j < costmap_width; j++)
            {
                costmap_2D[i][j] = 0;
            }
        }
        
        RCLCPP_INFO(logger_, "Costmap initialization completed");
    }
}