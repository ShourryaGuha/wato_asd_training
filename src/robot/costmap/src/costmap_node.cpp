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
      float x_grid_sensor_frame, y_grid_sensor_frame;  // in metres
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

void CostmapNode::inflateObstacles() {
  int inflation_radius_cells = inflation_radius_ / costmap_resolution_;
  
  // Create a copy of the original costmap to avoid modifying during iteration
  std::vector<std::vector<int>> original_costmap = costmap_2D;

  for(int i = 0; i < costmap_height; i++)
  {
    for(int j = 0; j < costmap_width; j++)
    {
      // Check if this cell is an obstacle (100 = occupied)
      if(original_costmap[i][j] == 100) 
      {
        // Inflate around this obstacle
        for(int dy = -inflation_radius_cells; dy <= inflation_radius_cells; dy++) {
          for(int dx = -inflation_radius_cells; dx <= inflation_radius_cells; dx++) {
            
            // Calculate neighboring cell coordinates
            int nx = j + dx;  // j is x-coordinate (column)
            int ny = i + dy;  // i is y-coordinate (row)

            // Check bounds
            if (nx >= 0 && nx < costmap_width && ny >= 0 && ny < costmap_height) {
              
              // Calculate Euclidean distance in cells
              float cell_distance = std::sqrt(dx*dx + dy*dy);
              
              // Only inflate within the radius
              if (cell_distance <= inflation_radius_cells) {
                
                // Calculate cost based on distance (closer = higher cost)
                // Cost decreases linearly from 99 at obstacle to ~20 at radius edge
                int inflation_cost;
                if (cell_distance == 0) {
                  inflation_cost = 100; // Keep obstacle at 100
                } else {
                  // Linear decrease: 99 at distance 1 to ~20 at max radius
                  inflation_cost = 99 - (int)((cell_distance / inflation_radius_cells) * 79);
                  inflation_cost = std::max(inflation_cost, 20); // Minimum inflation cost
                }
                
                // Only update if current cost is lower (don't overwrite obstacles or higher costs)
                if (costmap_2D[ny][nx] < inflation_cost) {
                  costmap_2D[ny][nx] = inflation_cost;
                }
              }
            }
          }
        }
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Obstacle inflation completed with radius: %d cells (%.1fm)", 
              inflation_radius_cells, inflation_radius_);
}


void CostmapNode::publishCostmap() {
  costmap_msg_.data.resize(costmap_height * costmap_width);
  for (int y = 0; y < costmap_height; ++y) {
    for (int x = 0; x < costmap_width; ++x) {
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