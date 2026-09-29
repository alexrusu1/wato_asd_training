#include "costmap_core.hpp"
#include <cmath>
#include <vector>
#include <cstdint>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initializeCostmap(){
    grid_.assign(height_, std::vector<int8_t>(width_, 0));
}

void CostmapCore::markObstacle(int x_grid, int y_grid){
    if(0 > x_grid || x_grid >= width_ || 0 > y_grid || y_grid >= height_) return;

    grid_[y_grid][x_grid] = 100;
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid){
    double x = range*cos(angle);
    double y = range*sin(angle);

    x_grid = (x - origin_x_) / resolution_;
    y_grid = (y - origin_y_) / resolution_;
}

void CostmapCore::inflateObstacles(){
    for(int i = 0; i < height_; i++){
        for(int j = 0; j < width_; j++){
            if(grid_[i][j] == 100){
                int radius_cells = inflation_radius_ / resolution_;
                for(int dy = -radius_cells; dy < radius_cells; dy ++){
                    for(int dx = -radius_cells; dx < radius_cells; dx ++){
                        int y_grid = i + dy;
                        int x_grid = j + dx;
                        double d = sqrt(dx*dx + dy*dy);
                        if(0 > x_grid || x_grid >= width_ || 0 > y_grid || y_grid >= height_ || d > radius_cells) continue;

                        double inflated_value = max_cost_ * (1 - d/radius_cells);
                        if(inflated_value > grid_[y_grid][x_grid]) grid_[y_grid][x_grid] = inflated_value;
                    }
                }
            }
        }
    }
}

std::vector<int8_t> CostmapCore::getGrid() const{
    std::vector<int8_t> new_grid;
    for(int i = 0; i < height_; i++){
        for(int j = 0; j < width_; j++){
            new_grid.push_back(grid_[i][j]);
        }
    }
    return new_grid;
}

double CostmapCore::getResolution() const { return resolution_; }
int CostmapCore::getWidth() const { return width_; }
int CostmapCore::getHeight() const { return height_; }
double CostmapCore::getOriginX() const { return origin_x_; }
double CostmapCore::getOriginY() const { return origin_y_; }

}