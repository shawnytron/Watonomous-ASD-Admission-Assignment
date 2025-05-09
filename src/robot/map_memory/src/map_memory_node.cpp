#include "map_memory_node.hpp"
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory_node"),
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    last_x_(0.0),
    last_y_(0.0),
    robot_theta_(0.0),
    costmap_received_(false),
    should_update_map_(false)
{
  int pub_rate = 500; // milliseconds
  map_memory_.initGlobalMap(global_map_);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(pub_rate),
    std::bind(&MapMemoryNode::updateMap, this));

  RCLCPP_INFO(this->get_logger(), "MapMemoryNode initialized.");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // Convert quaternion to yaw
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  robot_theta_ = quaternionToYaw(qx, qy, qz, qw);

  double distance = std::hypot(x - last_x_, y - last_y_);
  if (distance >= 1.5) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_received_) {
    map_memory_.mergeCostmap(global_map_, latest_costmap_, last_x_, last_y_, robot_theta_);

    global_map_.header.stamp = this->now();
    global_map_.header.frame_id = "sim_world";

    map_pub_->publish(global_map_);
    should_update_map_ = false;

    RCLCPP_INFO(this->get_logger(), "Published updated global map.");
  }
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w) {
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
