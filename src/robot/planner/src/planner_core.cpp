#include "planner_core.hpp"
#include <limits>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

void PlannerCore::setMap(const nav_msgs::msg::OccupancyGrid &map,
                         int occ_threshold,
                         bool unknown_as_free,
                         bool use_8_connected)
{
  map_ = map;
  has_map_ = true;
  occ_threshold_ = occ_threshold;
  unknown_free_  = unknown_as_free;
  use_8_         = use_8_connected;

  RCLCPP_DEBUG(logger_, "Map set: %ux%u res=%.3f origin(%.2f, %.2f)",
               map_.info.width, map_.info.height, map_.info.resolution,
               map_.info.origin.position.x, map_.info.origin.position.y);
}

std::optional<CellIndex> PlannerCore::worldToMap(double wx, double wy) const
{
  if (!has_map_) return std::nullopt;

  const double res = map_.info.resolution;
  const double ox  = map_.info.origin.position.x;
  const double oy  = map_.info.origin.position.y;

  const int mx = static_cast<int>(std::floor((wx - ox) / res));
  const int my = static_cast<int>(std::floor((wy - oy) / res));

  CellIndex c{mx, my};
  if (!inBounds(c)) return std::nullopt;
  return c;
}
geometry_msgs::msg::PoseStamped PlannerCore::mapToWorldPose(const CellIndex &c) const
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";

  const double res = map_.info.resolution;
  const double ox  = map_.info.origin.position.x;
  const double oy  = map_.info.origin.position.y;

  p.pose.position.x = ox + (c.x + 0.5) * res;
  p.pose.position.y = oy + (c.y + 0.5) * res;
  p.pose.orientation.w = 1.0;
  return p;
}

bool PlannerCore::inBounds(const CellIndex &c) const
{
  return has_map_ &&
         c.x >= 0 && c.y >= 0 &&
         c.x < static_cast<int>(map_.info.width) &&
         c.y < static_cast<int>(map_.info.height);
}

bool PlannerCore::isFree(const CellIndex &c) const
{
  if (!inBounds(c)) return false;
  const int v = map_.data[index(c)];  // -1 unknown, 0..100 occupancy
  if (v == -1) return unknown_free_;
  return v < occ_threshold_;
}
double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) const
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx*dx + dy*dy);
}

std::vector<CellIndex> PlannerCore::neighbors(const CellIndex &c) const
{
  static const int dx4[4] = { 1, -1,  0,  0};
  static const int dy4[4] = { 0,  0,  1, -1};
  static const int dx8[8] = { 1, -1,  0,  0,  1,  1, -1, -1};
  static const int dy8[8] = { 0,  0,  1, -1,  1, -1,  1, -1};

  std::vector<CellIndex> out;
  if (use_8_) {
    for (int i = 0; i < 8; ++i) {
      CellIndex n{c.x + dx8[i], c.y + dy8[i]};
      if (isFree(n)) out.push_back(n);
    }
  } else {
    for (int i = 0; i < 4; ++i) {
      CellIndex n{c.x + dx4[i], c.y + dy4[i]};
      if (isFree(n)) out.push_back(n);
    }
  }
  return out;
}
std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
    const CellIndex &start, const CellIndex &goal) const
{
  std::vector<CellIndex> cells;
  CellIndex cur = goal;
  cells.push_back(cur);

  while (cur != start) {
    auto it = came_from.find(cur);
    if (it == came_from.end()) break;  // safety
    cur = it->second;
    cells.push_back(cur);
  }
  std::reverse(cells.begin(), cells.end());

  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.reserve(cells.size());
  for (const auto &c : cells) path.push_back(mapToWorldPose(c));
  return path;
}
std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::plan(double sx, double sy,
                                                               double gx, double gy)
{
  std::vector<geometry_msgs::msg::PoseStamped> empty;
  if (!has_map_) {
    RCLCPP_WARN(logger_, "plan(): no map set");
    return empty;
  }

  auto s_opt = worldToMap(sx, sy);
  auto g_opt = worldToMap(gx, gy);
  if (!s_opt || !g_opt) {
    RCLCPP_WARN(logger_, "plan(): start/goal out of bounds");
    return empty;
  }

  CellIndex start = *s_opt;
  CellIndex goal  = *g_opt;

  if (!isFree(start) || !isFree(goal)) {
    RCLCPP_WARN(logger_, "plan(): start or goal not free (unknown_free=%s)",
                unknown_free_ ? "true" : "false");
    return empty;
  }

  // A* algorithm
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_set<CellIndex, CellIndexHash> closed;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start, goal));

  while (!open.empty()) {
    const CellIndex current = open.top().index;
    open.pop();

    if (closed.count(current)) continue;
    closed.insert(current);

    if (current == goal) {
      auto path = reconstructPath(came_from, start, goal);
      RCLCPP_DEBUG(logger_, "A* success: %zu poses", path.size());
      return path;
    }

    for (const auto &nbr : neighbors(current)) {
      const double step = (nbr.x != current.x && nbr.y != current.y) ? std::sqrt(2.0) : 1.0;
      const double tentative_g = g_score[current] + step;

      if (!g_score.count(nbr) || tentative_g < g_score[nbr]) {
        came_from[nbr] = current;
        g_score[nbr] = tentative_g;
        const double f = tentative_g + heuristic(nbr, goal);
        open.emplace(nbr, f);
      }
    }
  }

  RCLCPP_WARN(logger_, "A*: failed to find a path");
  return empty;
}

} 
