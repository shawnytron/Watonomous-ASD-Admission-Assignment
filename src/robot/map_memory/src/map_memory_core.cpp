#include "map_memory_core.hpp"
#include <algorithm>

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger), resolution_(0.1), width_(300), height_(300) {}

void MapMemoryCore::initGlobalMap(nav_msgs::msg::OccupancyGrid& map) {
  map.info.resolution = resolution_;
  map.info.width = width_;
  map.info.height = height_;
  map.info.origin.position.x = -15.0;
  map.info.origin.position.y = -15.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(width_ * height_, 0);

  RCLCPP_INFO(logger_, "Initialized global map [%d x %d] at resolution %.2f", width_, height_, resolution_);
}

void MapMemoryCore::mergeCostmap(
  nav_msgs::msg::OccupancyGrid& global_map,
  const nav_msgs::msg::OccupancyGrid& local_costmap,
  double robot_x, double robot_y, double robot_theta)
{
  const double local_res = local_costmap.info.resolution;
  const double local_origin_x = local_costmap.info.origin.position.x;
  const double local_origin_y = local_costmap.info.origin.position.y;
  const int local_w = local_costmap.info.width;
  const int local_h = local_costmap.info.height;

  const double cos_t = std::cos(robot_theta);
  const double sin_t = std::sin(robot_theta);

  for (int y = 0; y < local_h; ++y) {
    for (int x = 0; x < local_w; ++x) {
      int index = y * local_w + x;
      int8_t occ_val = local_costmap.data[index];
      if (occ_val <= 0) continue;

      // Convert local cell center to local coordinates
      double local_x = (x + 0.5) * local_res + local_origin_x;
      double local_y = (y + 0.5) * local_res + local_origin_y;

      // Transform to global (world) coordinates
      double world_x = robot_x + (local_x * cos_t - local_y * sin_t);
      double world_y = robot_y + (local_x * sin_t + local_y * cos_t);

      // Convert world coordinates to global map indices
      int gx = static_cast<int>((world_x - global_map.info.origin.position.x) / global_map.info.resolution);
      int gy = static_cast<int>((world_y - global_map.info.origin.position.y) / global_map.info.resolution);

      if (gx < 0 || gx >= static_cast<int>(global_map.info.width) ||
          gy < 0 || gy >= static_cast<int>(global_map.info.height)) {
        continue;
      }

      int global_index = gy * global_map.info.width + gx;
      int existing_val = global_map.data[global_index];
      global_map.data[global_index] = std::max(existing_val, static_cast<int>(occ_val));
    }
  }
}
}

