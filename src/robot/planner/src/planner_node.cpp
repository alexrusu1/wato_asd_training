#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

double PlannerNode::extractYaw(double qx, double qy, double qz, double qw) const {
    return atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    map_received_ = true;

    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_yaw_ = extractYaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
}

void PlannerNode::timerCallback() {
    if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

    if (goalReached()) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        state_ = State::WAITING_FOR_GOAL;
    } else {
        planPath();
    }
}

bool PlannerNode::goalReached() const {
    double dx = goal_.point.x - robot_x_;
    double dy = goal_.point.y - robot_y_;
    return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
    if (!goal_received_ || !map_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map or goal");
        return;
    }

    std::vector<std::pair<double, double>> path_points;
    bool found = planner_.planPath(
        current_map_.data,
        current_map_.info.width,
        current_map_.info.height,
        current_map_.info.resolution,
        current_map_.info.origin.position.x,
        current_map_.info.origin.position.y,
        robot_x_, robot_y_,
        goal_.point.x, goal_.point.y,
        path_points);

    if (!found) {
        RCLCPP_WARN(this->get_logger(), "Failed to find a path to the goal");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";

    for (const auto& [x, y] : path_points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
