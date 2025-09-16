#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

struct CellIndex
{
  int x;
  int y;

  CellIndex() = default;
  CellIndex(int x, int y) : x(x), y(y) {}

  bool operator==(const CellIndex &o) const { return x == o.x && y == o.y; }
  bool operator!=(const CellIndex &o) const { return !(*this == o); }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &ci) const
  {
    return std::hash<int>()(ci.x) ^ (std::hash<int>()(ci.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score{0.0};
  AStarNode() = default;
  AStarNode(CellIndex i, double f) : index(i), f_score(f) {}

  AStarNode(const CellIndex &index, double g_cost, double h_cost)
      : index(index), f_score(g_cost + h_cost) {}

  bool operator>(const AStarNode &other) const { return f_score > other.f_score; }
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b) const { return a.f_score > b.f_score; }
};

// PlannerCore class definition

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // config map + planning behaviour

  private:
    rclcpp::Logger logger_;
};

}  

#endif  
