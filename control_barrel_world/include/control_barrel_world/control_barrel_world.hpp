/**
 * @file control_barrel_world.hpp
 * @brief ROS2 Node for controlling the Mirte robot in the barrel world.
 * 
 * Subscribes to obstacle detections (/detections) and pedestrian detections (/pedestrians),
 * transforms 3D detections from the camera frame to the robot’s base frame (using TF2),
 * and publishes velocity commands on /mirte/cmd_vel to navigate safely.
 */


#ifndef CONTROL_BARREL_WORLD_HPP_
#define CONTROL_BARREL_WORLD_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>


class ControlBarrelWorld : public rclcpp::Node
{
public:
  /** Constructor – sets up subscriptions, publisher, and TF2. */
  ControlBarrelWorld();

private:
  /** Callback for 3D obstacle detections. */
  void obstacleCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

  /** Callback for 2D pedestrian detections. */
  void pedestrianCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  
  /** Callback: Main control logic combining all sensor data to decide robot motion. */
  void computeCommand();

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_obstacles_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_pedestrians_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  bool person_detected_;
  vision_msgs::msg::Detection3DArray::SharedPtr last_obstacles_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Periodic control loop timer
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // CONTROL_BARREL_WORLD_HPP_



