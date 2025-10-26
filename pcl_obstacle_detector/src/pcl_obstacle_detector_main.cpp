/**
 * @file pcl_obstacle_detector_main.cpp
 * @brief Entry point for pcl_obstacle_detector_node.
 * 
 * Initializes ROS 2, instantiates the PclObstacleDetector class,
 * and spins the node until shutdown.
 */

#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PclObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}




