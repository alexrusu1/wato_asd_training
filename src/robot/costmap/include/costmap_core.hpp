#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cstdint>

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap();
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    std::vector<int8_t> getGrid() const;

    double getResolution() const;
    int getWidth() const;
    int getHeight() const;
    double getOriginX() const;
    double getOriginY() const;

  private:
    rclcpp::Logger logger_;

    double resolution_ = 0.1;
    int width_ = 300;
    int height_ = 300;
    double origin_x_ = -15.0;
    double origin_y_ = -15.0;

    double inflation_radius_ = 1.2;
    int max_cost_ = 100;

    std::vector<std::vector<int8_t>> grid_;
};

}

#endif