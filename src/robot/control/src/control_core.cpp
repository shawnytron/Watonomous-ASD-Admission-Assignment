#include "control_core.hpp"

namespace robot {

ControlCore::ControlCore(double lookahead_distance, double goal_tolerance, double linear_speed)
  : lookahead_distance_(lookahead_distance),
    goal_tolerance_(goal_tolerance),
    linear_speed_(linear_speed) {}
//calc distance 
double ControlCore::computeDistance(const geometry_msgs::msg::Point& a,
                                    const geometry_msgs::msg::Point& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

//quat to yaw 
double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) const {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Pose& current_pose) const {
  for (const auto& pose_stamped : path.poses) {
    if (computeDistance(pose_stamped.pose.position, current_pose.position) > lookahead_distance_) {
      return pose_stamped;
    }
  }
  return std::nullopt;
}
//normal angle to -pi,pi
double ControlCore::normalizeAngle(double angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose) const {
  geometry_msgs::msg::Twist cmd_vel;

  double dx = target_pose.pose.position.x - current_pose.position.x;
  double dy = target_pose.pose.position.y - current_pose.position.y;

  double target_angle = std::atan2(dy, dx);
  double current_yaw = extractYaw(current_pose.orientation);
  
  double angle_error = normalizeAngle(target_angle - current_yaw);
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = 1.5 * angle_error; 
  return cmd_vel;
}

bool ControlCore::isGoalReached(const geometry_msgs::msg::Pose& robot_pose,
                                const geometry_msgs::msg::Pose& goal_pose) const {
  return computeDistance(robot_pose.position, goal_pose.position) < goal_tolerance_;
}

} 
