#include "planner_core.hpp"
#include <unordered_map>
#include <queue>
#include <cmath>

nav_msgs::msg::Path PlannerCore::aStarPlan(
  const nav_msgs::msg::OccupancyGrid& map,
  const geometry_msgs::msg::Pose& start,
  const geometry_msgs::msg::Point& goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "sim_world";

  int width = map.info.width;
  int height = map.info.height;
  double resolution = map.info.resolution;
  double origin_x = map.info.origin.position.x;
  double origin_y = map.info.origin.position.y;
  //Convert from real world coordnate to grid index lambda function
  auto poseToIndex = [&](double x, double y) -> CellIndex {
    return CellIndex(
      static_cast<int>((x - origin_x) / resolution),
      static_cast<int>((y - origin_y) / resolution)
    );
  };
  //reverse of above function 
  auto indexToPose = [&](const CellIndex& index) -> geometry_msgs::msg::PoseStamped {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "sim_world";
    pose.pose.position.x = index.x * resolution + origin_x + resolution / 2.0; //centers pose
    pose.pose.position.y = index.y * resolution + origin_y + resolution / 2.0; //centers pose
    pose.pose.orientation.w = 1.0;
    return pose;
  };
  //check free cell in bounds and = 0
  auto isFree = [&](int x, int y) -> bool {
    if (x < 0 || y < 0 || x >= width || y >= height) return false;
    int idx = y * width + x;
    return map.data[idx] == 0;
  };
  //manhattan distance
  auto heuristic = [&](const CellIndex& a, const CellIndex& b) -> double {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
  };
  //Convert to grid
  CellIndex start_idx = poseToIndex(start.position.x, start.position.y);
  CellIndex goal_idx = poseToIndex(goal.x, goal.y);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set; //lowest f score on top
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> visited;

  g_score[start_idx] = 0.0;
  //push start cell into openset
  open_set.emplace(start_idx, heuristic(start_idx, goal_idx));
  //8directions including diag
  std::vector<std::pair<int, int>> directions {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {-1, -1}, {1, -1}, {-1, 1}
  };

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal_idx) {
      // Reconstruct path
      CellIndex idx = goal_idx;
      while (idx != start_idx) {
        path.poses.push_back(indexToPose(idx));
        idx = came_from[idx];
      }
      path.poses.push_back(indexToPose(start_idx));
      std::reverse(path.poses.begin(), path.poses.end());
      return path;
    }

    visited[current.index] = true;

    for (const auto& dir : directions) {
      CellIndex neighbor(current.index.x + dir.first, current.index.y + dir.second);
      if (!isFree(neighbor.x, neighbor.y) || visited[neighbor]) continue;

      double tentative_g = g_score[current.index] + heuristic(current.index, neighbor);
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current.index;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal_idx);
        open_set.emplace(neighbor, f);
      }
    }
  }

  // If no path found
  return path;
}
