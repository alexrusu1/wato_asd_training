#include "control_core.hpp"
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
: logger_(logger) {}

double ControlCore::computeDistance(double x1, double y1, double x2, double y2) const {
    return std::sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

bool ControlCore::findLookaheadPoint(
    const std::vector<std::pair<double, double>>& path,
    double robot_x, double robot_y,
    std::pair<double, double>& lookahead_point_out) const {
        
        if(path.empty()) return false;

        for(const auto& point : path){
            if(computeDistance(robot_x, robot_y, point.first, point.second) >= lookahead_distance_){
                lookahead_point_out = {point.first, point.second};
                return true;
            }
        }
        lookahead_point_out = {path.back().first, path.back().second};
        return true;
}

void ControlCore::computeVelocity(
    double robot_x, double robot_y, double robot_yaw,
    const std::pair<double, double>& lookahead_point,
    double& linear_vel_out, double& angular_vel_out) const {

        double dx = lookahead_point.first - robot_x;
        double dy = lookahead_point.second - robot_y;
        double local_x = dx * cos(-robot_yaw) - dy * sin(-robot_yaw);
        double local_y = dx * sin(-robot_yaw) + dy * cos(-robot_yaw);

        double curvature = 2 * local_y / (lookahead_distance_ * lookahead_distance_);

        linear_vel_out = linear_speed_;
        angular_vel_out = curvature * linear_vel_out;
}

}
