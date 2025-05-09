#include "costmap_core.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger), inflation_radius_(1.0), resolution_(0.1) {} //per the wiki 

void CostmapCore::initCostmap(
    nav_msgs::msg::OccupancyGrid& costmap,
    double resolution,
    int width,
    int height,
    const geometry_msgs::msg::Pose& origin,
    double inflation_radius) {

  resolution_ = resolution;
  inflation_radius_ = inflation_radius;

  costmap.info.resolution = resolution;
  costmap.info.width = width;
  costmap.info.height = height;
  costmap.info.origin = origin;
// Initialize all cells to unknown (free space)
  costmap.data.assign(width * height, 0);

  RCLCPP_INFO(logger_, "Initialized costmap with size [%d x %d] and resolution %.2f", width, height, resolution);
}

void CostmapCore::updateCostmap(
    nav_msgs::msg::OccupancyGrid& costmap,
    const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
  //clear map to 0 
  for (size_t i = 0; i < costmap.data.size(); ++i) {
    costmap.data[i] = 0;
  }
    double angle = scan->angle_min;

  for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
    double r = scan->ranges[i];
    if (r < scan->range_min || r > scan->range_max) continue;

    // Convert the polar coordinates (range and angle) from the laser scan to Cartesian coordinates (x, y)
    double x = r * std::cos(angle);
    double y = r * std::sin(angle);
    
    // Convert the coordinates from the robot's frame to the costmap's frame
    int gx = static_cast<int>((x - costmap.info.origin.position.x) / resolution_);
    int gy = static_cast<int>((y - costmap.info.origin.position.y) / resolution_);

    //check if out of bound on map
    if (gx >= 0 && gx < static_cast<int>(costmap.info.width) &&
        gy >= 0 && gy < static_cast<int>(costmap.info.height)) {
      int idx = gy * costmap.info.width + gx;
      costmap.data[idx] = 100;

      inflateObstacle(costmap, gx, gy);
    }
  }
}

void CostmapCore::inflateObstacle(nav_msgs::msg::OccupancyGrid& costmap, int ox, int oy) {
    int width = costmap.info.width;
    int height = costmap.info.height;
  
    // How many cells around the obstacle to check
    int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
    int max_cost = 100;
  
    // Iterate in a square window centered at (ox, oy)
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        int nx = ox + dx;
        int ny = oy + dy;
  
        // Skip if out of map bounds
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
  
        // Calculate Euclidean distance in meters
        double distance = std::hypot(dx, dy) * resolution_;
        if (distance > inflation_radius_) continue;  // Outside radius, skip
  
        // Compute cost based on distance
        int cost = static_cast<int>(max_cost * (1.0 - (distance / inflation_radius_)));
  
        // Index into 1D costmap
        int idx = ny * width + nx;
  
        // Only apply cost if it's higher than what's already there
        costmap.data[idx] = std::max(costmap.data[idx], static_cast<int8_t>(cost));
    }
    }
  }
  

}
