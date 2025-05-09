#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot {

class MapMemoryCore {
public:
  explicit MapMemoryCore(const rclcpp::Logger& logger);

  void initGlobalMap(nav_msgs::msg::OccupancyGrid& map);
  void mergeCostmap(
    nav_msgs::msg::OccupancyGrid& global_map,
    const nav_msgs::msg::OccupancyGrid& costmap,
    double robot_x, double robot_y, double robot_theta);
private:
  rclcpp::Logger logger_;
  double resolution_;
  int width_;
  int height_;
};

}  

#endif
