#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));

  global_map_ = nav_msgs::msg::OccupancyGrid();
  latest_costmap_ = nav_msgs::msg::OccupancyGrid();

  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "map";
  global_map_.info.resolution = RES;
  global_map_.info.width = COST_MAP_WIDTH;
  global_map_.info.height = COST_MAP_HEIGHT;
  global_map_.info.origin.position.x = -COST_MAP_WIDTH * RES / 2.0;
  global_map_.info.origin.position.y = -COST_MAP_HEIGHT * RES / 2.0;
  global_map_.data.assign(COST_MAP_WIDTH * COST_MAP_HEIGHT, 0);
}

// Callback for costmap updates
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Store the latest costmap
  latest_costmap_ = *msg;
  updated_costmap_ = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
