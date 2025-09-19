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
    void initializeCostmap(std::vector<std::vector<int>>& costmap_2D, int costmap_width, int costmap_height);
  private:
    rclcpp::Logger logger_;

    

};

}  

#endif  