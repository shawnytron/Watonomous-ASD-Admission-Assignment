#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <optional>
#include <cmath>

namespace robot {

class ControlCore {
public:
  ControlCore(double lookahead_distance, double goal_tolerance, double linear_speed);

  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
    const nav_msgs::msg::Path& path, const geometry_msgs::msg::Pose& current_pose) const;

  geometry_msgs::msg::Twist computeVelocity(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose) const;

  bool isGoalReached(const geometry_msgs::msg::Pose& robot_pose,
                     const geometry_msgs::msg::Pose& goal_pose) const;

private:
  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;
  double normalizeAngle(double angle) const;

  double computeDistance(const geometry_msgs::msg::Point& a,
                         const geometry_msgs::msg::Point& b) const;

  double extractYaw(const geometry_msgs::msg::Quaternion& q) const;
};

}  

#endif  