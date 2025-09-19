#include <chrono>
#include <memory>

#include "costmap_node.hpp"
#include "costmap_core.hpp"

CostmapNode::CostmapNode() : Node("costmap"),
                             costmap_(robot::CostmapCore(this->get_logger())),
                             costmap_2D(map_width_, std::vector<int>(map_height_, 0))
{
  // Initialize publishers
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // Initialize subscribers
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  initializeCostmap();
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::initializeCostmap()
{
  // use resolution and dimensions to calculate 2D array size
  int costmap_width = (int)(map_width_ / costmap_resolution_);
  int costmap_height = (int)(map_height_ / costmap_resolution_);

  // initialize costmap message static parts
  costmap_msg_.info.resolution = costmap_resolution_; // meters per cell
  costmap_msg_.info.width = costmap_width;
  costmap_msg_.info.height = costmap_height;

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
  // Step 1: Initialize costmap done in constructor

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min)
    {
      // Calculate grid coordinates
      float x_grid_sensor_frame, y_grid_sensor_frame;
      costmap_.convertToGrid(range, angle, x_grid_sensor_frame, y_grid_sensor_frame);

      int x_grid = static_cast<int>(x_grid_sensor_frame / costmap_resolution_ + (costmap_width / 2));
      int y_grid = static_cast<int>(y_grid_sensor_frame / costmap_resolution_ + (costmap_height / 2));

      costmap_.markObstacle(costmap_2D, x_grid, y_grid);
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