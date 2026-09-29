#include <cmath>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  map_memory_.initializeGlobalMap();

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

double MapMemoryNode::extractYaw(double qx, double qy, double qz, double qw){
    return atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    latest_costmap_ = *msg;
    costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    robot_yaw_ = extractYaw(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    double distance = sqrt(pow(robot_x_ - last_x_, 2) + pow(robot_y_ - last_y_, 2));
    if(distance >= distance_threshold_){
        last_x_ = robot_x_;
        last_y_ = robot_y_;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap(){
    if(should_update_map_ && costmap_received_){
        map_memory_.updateMap(
            latest_costmap_.data,
            latest_costmap_.info.resolution,
            latest_costmap_.info.width,
            latest_costmap_.info.height,
            latest_costmap_.info.origin.position.x,
            latest_costmap_.info.origin.position.y,
            robot_x_, robot_y_, robot_yaw_);

        auto map_msg = nav_msgs::msg::OccupancyGrid();
        map_msg.header.stamp = this->get_clock()->now();
        map_msg.header.frame_id = "sim_world";

        map_msg.info.resolution = map_memory_.getResolution();
        map_msg.info.width = map_memory_.getWidth();
        map_msg.info.height = map_memory_.getHeight();
        map_msg.info.origin.position.x = map_memory_.getOriginX();
        map_msg.info.origin.position.y = map_memory_.getOriginY();

        map_msg.data = map_memory_.getGrid();

        map_pub_->publish(map_msg);

        should_update_map_ = false;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
