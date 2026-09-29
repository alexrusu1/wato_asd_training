#include <cmath>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

double ControlNode::extractYaw(double qx, double qy, double qz, double qw) const {
    return atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_.clear();
    for (const auto& pose : msg->poses) {
        current_path_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    path_received_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_yaw_ = extractYaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    odom_received_ = true;
}

void ControlNode::controlLoop() {
    if (!path_received_ || !odom_received_ || current_path_.empty()) return;

    const auto& goal = current_path_.back();
    double dist_to_goal = std::sqrt(
        std::pow(goal.first - robot_x_, 2) + std::pow(goal.second - robot_y_, 2));

    geometry_msgs::msg::Twist cmd_vel;

    if (dist_to_goal < goal_tolerance_) {
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    std::pair<double, double> lookahead_point;
    if (!control_.findLookaheadPoint(current_path_, robot_x_, robot_y_, lookahead_point)) {
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    double linear_vel = 0.0;
    double angular_vel = 0.0;
    control_.computeVelocity(robot_x_, robot_y_, robot_yaw_, lookahead_point, linear_vel, angular_vel);

    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
    cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
