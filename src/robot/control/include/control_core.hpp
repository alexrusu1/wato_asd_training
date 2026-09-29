#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <utility>

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    bool findLookaheadPoint(
        const std::vector<std::pair<double, double>>& path,
        double robot_x, double robot_y,
        std::pair<double, double>& lookahead_point_out) const;

    void computeVelocity(
        double robot_x, double robot_y, double robot_yaw,
        const std::pair<double, double>& lookahead_point,
        double& linear_vel_out, double& angular_vel_out) const;

  private:
    double computeDistance(double x1, double y1, double x2, double y2) const;

    rclcpp::Logger logger_;

    const double lookahead_distance_ = 0.6;
    const double linear_speed_ = 0.3;
};

}

#endif
