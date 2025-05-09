#include "control_node.hpp"

namespace robot {

ControlNode::ControlNode() : Node("control_node") {
  double lookahead_distance = 1.5;
  double goal_tolerance = 0.5;
  double linear_speed = 0.5;

  controller_ = std::make_unique<ControlCore>(lookahead_distance, goal_tolerance, linear_speed);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "ControlNode initialized.");
}
//callback below
void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = msg->pose.pose;
}

void ControlNode::controlLoop() {
  if (!current_path_ || current_path_->poses.empty()) return;

  const geometry_msgs::msg::Pose& goal_pose = current_path_->poses.back().pose;

  if (controller_->isGoalReached(current_pose_, goal_pose)) {
    RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping.");
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0; //stopping
    stop_msg.angular.z = 0.0; //stopping
    cmd_vel_pub_->publish(stop_msg);
    return;
  }

  auto lookahead = controller_->findLookaheadPoint(*current_path_, current_pose_);
  if (lookahead) {
    auto cmd = controller_->computeVelocity(current_pose_, *lookahead);
    cmd_vel_pub_->publish(cmd);
  }
}

} 

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot::ControlNode>());
  rclcpp::shutdown();
  return 0;
}
