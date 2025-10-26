/**
 * @file control_barrel_world_main.cpp
 * @brief Entry point for the control_barrel_world_node.
 * 
 * Initializes ROS 2, creates the ControlBarrelWorld node,
 * and starts spinning until shutdown.
 */

 
#include "rclcpp/rclcpp.hpp"
#include "control_barrel_world/control_barrel_world.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlBarrelWorld>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

