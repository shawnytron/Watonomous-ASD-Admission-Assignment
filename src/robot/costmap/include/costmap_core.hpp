#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot {

class CostmapCore {
public:
  explicit CostmapCore(const rclcpp::Logger& logger);

  // Initializes the given costmap in-place
  void initCostmap(
    nav_msgs::msg::OccupancyGrid& costmap,
    double resolution,
    int width,
    int height,
    const geometry_msgs::msg::Pose& origin,
    double inflation_radius);

  // Updates costmap with laser data
  void updateCostmap(
    nav_msgs::msg::OccupancyGrid& costmap,
    const sensor_msgs::msg::LaserScan::SharedPtr& scan);

  // Inflate cost around obstacle cell
  void inflateObstacle(nav_msgs::msg::OccupancyGrid& costmap, int ox, int oy);

private:
  rclcpp::Logger logger_;
  double inflation_radius_;
  double resolution_;
};

} // namespace robot

#endif
