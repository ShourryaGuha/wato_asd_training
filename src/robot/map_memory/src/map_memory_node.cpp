#include "map_memory_node.hpp"

#include <cmath>
#include <chrono>
using namespace std::chrono_literals;

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), map_memory_(this->get_logger()) {
  // 1) Params
  declareAndGetParams();

  // 2) Publisher (transient_local so late subscribers get the last map)
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", pub_qos);

  // 3) Subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", rclcpp::QoS(10),
    std::bind(&MapMemoryNode::onCostmap, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", rclcpp::QoS(50),
    std::bind(&MapMemoryNode::onOdom, this, std::placeholders::_1));

  // 4) Timer (e.g., 1 Hz)
  timer_ = this->create_wall_timer(1s, std::bind(&MapMemoryNode::timerCallback, this));

  // 5) Initialize global map in the core
  initGlobalMapViaCore();

  RCLCPP_INFO(this->get_logger(),
              "map_memory node ready. dist_thresh=%.2f, frame=%s, res=%.3f, size=%dx%d, origin=(%.2f,%.2f)",
              update_distance_threshold_, map_frame_id_.c_str(), resolution_,
              width_cells_, height_cells_, origin_x_, origin_y_);
}

void MapMemoryNode::declareAndGetParams() {
  update_distance_threshold_ = this->declare_parameter<double>("update_distance_threshold", update_distance_threshold_);
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", map_frame_id_);
  resolution_ = this->declare_parameter<double>("resolution", resolution_);
  width_cells_ = this->declare_parameter<int>("width_cells", width_cells_);
  height_cells_ = this->declare_parameter<int>("height_cells", height_cells_);
  origin_x_ = this->declare_parameter<double>("origin_x", origin_x_);
  origin_y_ = this->declare_parameter<double>("origin_y", origin_y_);
}

void MapMemoryNode::initGlobalMapViaCore() {
  map_memory_.initializeGlobalMap(map_frame_id_, resolution_, width_cells_, height_cells_, origin_x_, origin_y_);
}

void MapMemoryNode::onCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  latest_costmap_ = *msg;
  have_costmap_ = true;
}

void MapMemoryNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  latest_odom_ = *msg;

  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  const double dx = x - last_update_x_;
  const double dy = y - last_update_y_;
  const double dist = std::hypot(dx, dy);

  if (dist >= update_distance_threshold_) {
    should_update_ = true;
  }
}

void MapMemoryNode::timerCallback() {
  // Snapshot state under lock, then release to keep callbacks responsive
  nav_msgs::msg::OccupancyGrid costmap_snapshot;
  nav_msgs::msg::Odometry odom_snapshot;
  bool have_costmap = false;
  bool do_update = false;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    have_costmap = have_costmap_;
    do_update = should_update_;
    if (have_costmap && do_update) {
      costmap_snapshot = latest_costmap_;
      odom_snapshot = latest_odom_;
    }
  }

  if (!have_costmap || !do_update) {
    return;
  }

  // Extract robot pose in global frame
  const auto &p = odom_snapshot.pose.pose.position;
  const auto &q = odom_snapshot.pose.pose.orientation;
  const double yaw = yawFromQuat(q.x, q.y, q.z, q.w);

  // Fuse snapshot into the global map
  map_memory_.fuseCostmap(costmap_snapshot, p.x, p.y, yaw);

  // Publish the updated map (stamp here)
  auto map = map_memory_.globalMap();
  auto stamped = map;  // copy to stamp (globalMap() is const-ref)
  stamped.header.stamp = this->now();
  map_pub_->publish(stamped);

  // Reset gating after a successful update
  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_update_x_ = p.x;
    last_update_y_ = p.y;
    should_update_ = false;
  }
}

// Minimal yaw extraction (no tf2 needed)
double MapMemoryNode::yawFromQuat(double x, double y, double z, double w) {
  // yaw (Z axis rotation)
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
