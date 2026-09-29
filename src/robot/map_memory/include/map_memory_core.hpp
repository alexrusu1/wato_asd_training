#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cstdint>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initializeGlobalMap();

    void updateMap(
        const std::vector<int8_t>& local_grid,
        double local_resolution, int local_width, int local_height,
        double local_origin_x, double local_origin_y,
        double robot_x, double robot_y, double robot_yaw);

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

    std::vector<std::vector<int8_t>> global_grid_;
};

}

#endif
