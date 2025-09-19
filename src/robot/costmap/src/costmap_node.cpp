#include <chrono>
#include <memory>

#include "costmap_node.hpp"
#include "costmap_core.hpp"

CostmapNode::CostmapNode() : Node("costmap"),
                             costmap_(robot::CostmapCore(this->get_logger())),
                             costmap_2D(costmap_width_, std::vector<int>(costmap_height_, 0))
{
  // Initialize publishers
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // Initialize subscribers
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  initializeCostmap(costmap_width_, costmap_height_);
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::initializeCostmap(int costmap_width, int costmap_height)
{

  float costmap_resolution = 0.1; // meters per cell


  // use resolution and dimensions to calculate 2D array size
  int width_2d = (int)(costmap_width_ / costmap_resolution);
  int height_2d = (int)(costmap_height_ / costmap_resolution);

  // Resize the 2D vector to the specified dimensions
  costmap_2D.resize(costmap_height);
  for (int i = 0; i < costmap_height; i++)
  {
    costmap_2D[i].resize(costmap_width);
  }

  // Initialize all cells to 0 (free space)
  for (int i = 0; i < costmap_height; i++)
  {
    for (int j = 0; j < costmap_width; j++)
    {
      costmap_2D[i][j] = 0;
    }
  }

  // RCLCPP_INFO(logger_, "Costmap initialization completed");
}

// Callback function for lidar data
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Step 1: Initialize costmap

  float costmap_resolution = 0.1; // meters per cell

  // use resolution and dimensions to calculate 2D array size
  int width_2d = (int)(costmap_width_ / costmap_resolution);
  int height_2d = (int)(costmap_height_ / costmap_resolution);

  costmap_.initializeCostmap(costmap_2D, width_2d, height_2d);

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min)
    {
      // Calculate grid coordinates
      int x_grid, y_grid;
      // convertToGrid(range, angle, x_grid, y_grid);
      // markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: Inflate obstacles
  // inflateObstacles();

  // Step 4: Publish costmap
  // publishCostmap();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}