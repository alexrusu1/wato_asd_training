#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initializeGlobalMap(){
    global_grid_.resize(height_);
    for(int i = 0; i < height_; i++){
        global_grid_[i].resize(width_, 0);
    }
}

void MapMemoryCore::updateMap(
    const std::vector<int8_t>& local_grid,
    double local_resolution, int local_width, int local_height,
    double local_origin_x, double local_origin_y,
    double robot_x, double robot_y, double robot_yaw){
        for(int ly = 0; ly < local_height; ly++){
            for(int lx = 0; lx < local_width; lx++){
                int8_t value = local_grid[ly * local_width + lx];

                double local_x = lx * local_resolution + local_origin_x;
                double local_y = ly * local_resolution + local_origin_y;

                double global_x = robot_x + (local_x * cos(robot_yaw) - local_y * sin(robot_yaw));
                double global_y = robot_y + (local_x * sin(robot_yaw) + local_y * cos(robot_yaw));

                int gx = (global_x - origin_x_) / resolution_;
                int gy = (global_y - origin_y_) / resolution_;

                if(gx >= width_ || gx < 0 || gy >= height_ || gy < 0) continue;

                global_grid_[gy][gx] = value;
            }
        }
}

std::vector<int8_t> MapMemoryCore::getGrid() const{
    std::vector<int8_t> new_grid;
    for(int i = 0; i < height_; i++){
        for(int j = 0; j < width_; j++){
            new_grid.push_back(global_grid_[i][j]);
        }
    }
    return new_grid;
}

double MapMemoryCore::getResolution() const { return resolution_; }
int MapMemoryCore::getWidth() const { return width_; }
int MapMemoryCore::getHeight() const { return height_; }
double MapMemoryCore::getOriginX() const { return origin_x_; }
double MapMemoryCore::getOriginY() const { return origin_y_; }

}
