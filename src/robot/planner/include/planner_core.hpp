#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cstdint>
#include <utility>

namespace robot
{

struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    bool planPath(
        const std::vector<int8_t>& grid, int width, int height,
        double resolution, double origin_x, double origin_y,
        double start_x, double start_y,
        double goal_x, double goal_y,
        std::vector<std::pair<double, double>>& path_out);

  private:
    CellIndex worldToGrid(double x, double y, double resolution, double origin_x, double origin_y) const;
    std::pair<double, double> gridToWorld(const CellIndex& idx, double resolution, double origin_x, double origin_y) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    bool isFree(const CellIndex& idx, int width, int height, const std::vector<int8_t>& grid) const;

    rclcpp::Logger logger_;

    const int8_t occupancy_threshold_ = 50;
};

}

#endif
