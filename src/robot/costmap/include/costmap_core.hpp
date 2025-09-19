#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void convertToGrid(float range, float angle, float& x_grid, float& y_grid);

    void markObstacle(std::vector<std::vector<int>>& costmap_2D, int x_grid, int y_grid);
  private:
    rclcpp::Logger logger_;


};

}  

#endif  