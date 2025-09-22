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
    void setMap(const nav_msgs::msg::OccupancyGrid &map,
              int occ_threshold = 50,
              bool unknown_as_free = true,
              bool use_8_connected = true);

    std::vector<geometry_msgs::msg::PoseStamped> plan(double start_x, double start_y,
                                                     double goal_x, double goal_y);

    std::optional<CellIndex> worldToMap(double wx, double wy) const;
    geometry_msgs::msg::PoseStamped mapToWorldPose(const CellIndex &c) const;

  private:
    // map settings
    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_{false};
    int occ_threshold_{50};
    bool unknown_free_{true};
    bool use_8_{true};
    rclcpp::Logger logger_;

    // helpers
    bool inBounds(const CellIndex &c) const;
    bool isFree(const CellIndex &c) const;
    int index(const CellIndex &c) const { return c.y * static_cast<int>(map_.info.width) + c.x; }
    double heuristic(const CellIndex &a, const CellIndex &b) const; // Euclidean
    std::vector<CellIndex> neighbors(const CellIndex &c) const;
    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(
        const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
        const CellIndex &start, const CellIndex &goal) const;
};

}  

#endif  
