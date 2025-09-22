#include <chrono>
#include <memory>

#include "costmap_node.hpp"
#include "costmap_core.hpp"
#include <cmath>

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
  costmap_width = (int)(map_width_ / costmap_resolution_);
  costmap_height = (int)(map_height_ / costmap_resolution_);

  // initialize costmap message static parts
  costmap_msg_.info.resolution = costmap_resolution_; // meters per cell
  costmap_msg_.info.width = costmap_width;
  costmap_msg_.info.height = costmap_height;

  // Set the origin - position of the (0,0) cell in world coordinates
  // For a robot-centered map, place origin so robot is in the center
  costmap_msg_.info.origin.position.x = origin_x_;
  costmap_msg_.info.origin.position.y = origin_y_;

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
      float x_grid_sensor_frame, y_grid_sensor_frame; // in metres
      costmap_.convertToGrid(range, angle, x_grid_sensor_frame, y_grid_sensor_frame);

      int x_grid = static_cast<int>(x_grid_sensor_frame / costmap_resolution_ + (costmap_width / 2));
      int y_grid = static_cast<int>(y_grid_sensor_frame / costmap_resolution_ + (costmap_height / 2));

      costmap_.markObstacle(costmap_2D, x_grid, y_grid);
    }
  }

  // Step 3: Inflate obstacles
  inflateObstacles();

  // Step 4: Publish costmap

  publishCostmap();
}

void CostmapNode::inflateObstacles()
{
  int inflation_radius_cells = inflation_radius_ / costmap_resolution_;

  for (int i = 0; i < costmap_height; i++)
  {
    for (int j = 0; j < costmap_width; j++)
    {
      // Placeholder for obstacle inflation logic
      if (costmap_2D[i][j] == 100) // assuming 100 represents an obstacle
      {
        // Inflate logic here, e.g., mark neighboring cells
        for (int y = -1 * inflation_radius_cells; y <= inflation_radius_cells; y++)
        {
          for (int x = -1 * inflation_radius_cells; x <= inflation_radius_cells; x++)
          {

            // neighbouring cells
            int nx = j + x;
            int ny = i + y;

            if (nx >= 0 && nx < costmap_width && ny >= 0 && ny < costmap_height)
            {
              float euc_dist = std::sqrt((x * x + y * y));
              if (euc_dist > inflation_radius_cells)
              {
                continue;
              }
              int new_cost = 100 * (1 - euc_dist / inflation_radius_cells);
              costmap_2D[ny][nx] = (new_cost > costmap_2D[ny][nx]) ? new_cost : costmap_2D[ny][nx];
            }
            else
            {
              continue;
            }
          }
        }
      }
      else
      {
        continue;
      }
    }
  }
}

void CostmapNode::publishCostmap()
{
  costmap_msg_.data.resize(costmap_height * costmap_width);
  for (int y = 0; y < costmap_height; ++y)
  {
    for (int x = 0; x < costmap_width; ++x)
    {
      costmap_msg_.data[y * costmap_width + x] = costmap_2D[y][x];
    }
  }
  costmap_msg_.header.stamp = this->now();
  costmap_msg_.header.frame_id = "map";
  costmap_pub_->publish(costmap_msg_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}