#include "costmap_node.hpp"

CostmapNode::CostmapNode()
  : Node("costmap_node"),
    core_(robot::CostmapCore(this->get_logger())) {

  // Initialize costmap 
  double resolution = 0.1;
  int width = 300;
  int height = 300;
  double inflation_radius = 1.3;

  geometry_msgs::msg::Pose origin;
  origin.position.x = -15.0;
  origin.position.y = -15.0;
  origin.position.z = 0.0;
  origin.orientation.w = 1.0;

  core_.initCostmap(costmap_, resolution, width, height, origin, inflation_radius);

  // Subscriber and publisher created with inline topic names and queue sizes
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);


  RCLCPP_INFO(this->get_logger(), "Minimal CostmapNode initialized.");
}

void CostmapNode::laserCallback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  core_.updateCostmap(costmap_, msg);

  costmap_.header.stamp = msg->header.stamp;
  costmap_.header.frame_id = "map";

  costmap_pub_->publish(costmap_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
