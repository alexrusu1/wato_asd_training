#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
 
void CostmapNode::publishCostmap(){
  auto grid_msg = nav_msgs::msg::OccupancyGrid();

  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.header.frame_id = "robot/chassis/lidar";

  grid_msg.info.resolution = costmap_.getResolution();
  grid_msg.info.width = costmap_.getWidth();
  grid_msg.info.height = costmap_.getHeight();
  grid_msg.info.origin.position.x = costmap_.getOriginX();
  grid_msg.info.origin.position.y = costmap_.getOriginY();

  grid_msg.data = costmap_.getGrid();

  occupancy_grid_->publish(grid_msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    costmap_.initializeCostmap();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            int x_grid, y_grid;
            costmap_.convertToGrid(range, angle, x_grid, y_grid);
            costmap_.markObstacle(x_grid, y_grid);
        }
    }

    costmap_.inflateObstacles();
    publishCostmap();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}