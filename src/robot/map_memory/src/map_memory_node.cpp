#include "map_memory_node.hpp"
#include <functional>
using namespace std;

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
   // Initialize subscribers
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
 
        // Declare and get parameters
        declareAndGetParameters();

        // Initialize timer
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
        timer_ = this->create_wall_timer(
          update_period_, std::bind(&MapMemoryNode::timerCallback, this));

        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
}

// implementing the Callback functions 
void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg; // store the latest costmap
    have_latest_costmap_ = true; // set the flag to true
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Received costmap: %dx%d, resolution: %.3f", 
                        msg->info.width, msg->info.height, msg->info.resolution);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x; // update current x position
    current_y_ = msg->pose.pose.position.y; // update current y position

    // Extract yaw from quaternion
    double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                              msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + 
                                    msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp); // update current yaw
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Robot pose: x=%.2f, y=%.2f, yaw=%.2f", 
                        current_x_, current_y_, current_yaw_);
    
    if(!anchor_set_){
        markUpdateAnchorToCurrentPose();
        anchor_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Anchor set to initial position: x=%.2f, y=%.2f", 
                   current_x_, current_y_);
    }
}

void MapMemoryNode::timerCallback() {
  if (!have_latest_costmap_ || !anchor_set_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "Waiting for data: costmap=%s, anchor=%s", 
                        have_latest_costmap_ ? "✓" : "✗", 
                        anchor_set_ ? "✓" : "✗");
    return;
  }

  double distance = sqrt(pow(last_update_x_ - current_x_, 2) + pow(last_update_y_ - current_y_, 2));
  
  // Publish first map immediately, then check distance threshold for subsequent updates
  bool should_publish = !first_map_published_ || movedEnoughSinceLastUpdate();
  
  if (!should_publish) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "Not moved enough: distance=%.3f, threshold=%.3f", 
                        distance, distance_threshold_m_);
    return;
  }

  if (!first_map_published_) {
    RCLCPP_INFO(this->get_logger(), "Publishing initial map");
  } else {
    RCLCPP_INFO(this->get_logger(), "Fusing costmap and publishing map (moved %.3f m)", distance);
  }

  // 1) Fuse latest local costmap into the core's global map
  map_memory_.fuseCostmap(latest_costmap_, current_x_, current_y_, current_yaw_);

  // 2) Stamp and publish (getGlobalMap returns const&, so make a copy to set stamp)
  auto map = map_memory_.getGlobalMap();
  auto out = map;                      // copy so we can modify header
  out.header.stamp = this->now();
  out.header.frame_id = "sim_world";         // Set consistent frame_id
  map_pub_->publish(out);

  RCLCPP_INFO(this->get_logger(), "Published global map: %dx%d", out.info.width, out.info.height);

  // 3) Move the anchor to current pose and mark first map as published
  markUpdateAnchorToCurrentPose();
  first_map_published_ = true;
}


// implementing the Helper functions
bool MapMemoryNode::movedEnoughSinceLastUpdate() const {
    double x = last_update_x_ - current_x_;
    double y = last_update_y_ - current_y_;
    double distance = sqrt(x*x + y*y);
    return distance >= distance_threshold_m_;
}

void MapMemoryNode::markUpdateAnchorToCurrentPose() {
    last_update_x_ = current_x_;
    last_update_y_ = current_y_;
}

void MapMemoryNode::declareAndGetParameters() {
    this->declare_parameter("distance_threshold_m", 1.5);  // Keep onboarding requirement of 1.5m
    this->get_parameter("distance_threshold_m", distance_threshold_m_);

    int update_period_ms = 1000;
    this->declare_parameter("update_period_ms", update_period_ms);
    this->get_parameter("update_period_ms", update_period_ms);
    update_period_ = std::chrono::milliseconds(update_period_ms);
    
    RCLCPP_INFO(this->get_logger(), "Parameters: distance_threshold=%.2f m, update_period=%d ms", 
               distance_threshold_m_, update_period_ms);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}