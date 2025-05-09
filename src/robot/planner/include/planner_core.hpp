#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include <unordered_map>
#include <queue>

// ------------------- Supporting Structures -------------------

struct CellIndex {
  int x, y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex &other) const { return x != other.x || y != other.y; }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score; //f=g+h
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

// ------------------- PlannerCore Class -------------------

class PlannerCore {
public:
  static nav_msgs::msg::Path aStarPlan(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Point& goal);
};

#endif  // PLANNER_CORE_HPP_
