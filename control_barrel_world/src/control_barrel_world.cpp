/**
 * @file control_barrel_world.cpp
 * @brief Implements the autonomous control logic for the Mirte robot.
 *
 * The file defines:
 *   1. One node class: ControlBarrelWorld
 *   2. Three callback functions (class methods - member functions):
 *      - obstacleCallback() — reacts to /detections
 *      - pedestrianCallback() — reacts to /pedestrians
 *      - computeCommand() — computes and publishes velocity commands
 *
 * This node:
 * - Subscribes to /detections (3D obstacles) and /pedestrians (2D detections)
 * - Transforms 3D detections from camera frame to robot base_link frame using TF2
 * - Applies a rule-based control algorithm to avoid obstacles and stop for pedestrians
 */

#include "control_barrel_world/control_barrel_world.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using std::placeholders::_1;


ControlBarrelWorld::ControlBarrelWorld()
: Node("control_barrel_world_node"), person_detected_(false)
{
  // TF2 setup 
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions - // Queue size of subscriber is 2
  sub_obstacles_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    "/detections", 2, std::bind(&ControlBarrelWorld::obstacleCallback, this, _1));

  sub_pedestrians_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/pedestrians", 2, std::bind(&ControlBarrelWorld::pedestrianCallback, this, _1));

  // Publisher - // Queue size of subscriber is 2
  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/mirte/cmd_vel", 2);

  RCLCPP_INFO(this->get_logger(), "Control Barrel World node started with TF2 transform enabled");
}


// --- obstacle Callback --- 
/*
Triggered when: A new /detections message arrives from the pcl_obstacle_detector_node.
What it does:
  1. Stores the detections in last_obstacles_.
  2. Calls computeCommand() to decide how the robot should move.

Called whenever new obstacle detections arrive from the PCL node.
Updates internal obstacle data and triggers movement computation.
*/

void ControlBarrelWorld::obstacleCallback(
  const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  last_obstacles_ = msg;
  computeCommand();
}

// --- pedestrian Callback ---
/*
Triggered when: he OpenCV-based opencv_person_detector_node publishes 2D detections on /pedestrians.
What it does: 
  1. Checks if any detection has a bounding box area > 2500 pixels².
  2. If yes → sets person_detected_ = true and stops the robot permanently.
  3. Calls computeCommand() again to update movement.

Called whenever new pedestrian detections arrive.
If a person is detected with large bounding box, stop permanently.
*/

void ControlBarrelWorld::pedestrianCallback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (person_detected_) return;  // stop permanently once a person is seen

  for (auto &det : msg->detections) {
    double area = det.bbox.size_x * det.bbox.size_y;
    if (area > 2500.0) {
      person_detected_ = true;
      RCLCPP_WARN(this->get_logger(),
        "Pedestrian detected (area %.0f px²) → stopping permanently", area);
      break;
    }
  }
  computeCommand();
}

// ---------------- MAIN CONTROL LOGIC ----------------

// --- computeCommand Callback ---
/*
Triggered by: Both of the above callbacks (obstacleCallback and pedestrianCallback).
What it does:
  1. Reads the latest detections.
  2. Applies control logic to decide movement (forward / turn / stop).
  3. Publishes a /mirte/cmd_vel command (geometry_msgs::Twist).

Computes and publishes the robot's velocity command based on obstacle and pedestrian detections.
*/

void ControlBarrelWorld::computeCommand()
{
  geometry_msgs::msg::Twist cmd;

  // STEP 1: Stop permanently if a person was detected
  if (person_detected_) {
    cmd.linear.x = 0.0;         // Linear forward velocity
    cmd.angular.z = 0.0;        // Angular velocity
    pub_cmd_vel_->publish(cmd);
    return;
  }

  // STEP 2: If no detections then drive forward
  if (!last_obstacles_ || last_obstacles_->detections.empty()) {
    cmd.linear.x = 0.17;
    cmd.angular.z = 0.0;
    pub_cmd_vel_->publish(cmd);
    return;
  }

  // STEP 3: Transform detections from camera to base_link frame
  std::vector<geometry_msgs::msg::Point> valid_points;
  for (auto &det : last_obstacles_->detections) {
    geometry_msgs::msg::PointStamped p_in, p_out;
    p_in.header.frame_id = det.header.frame_id;   // usually "mirte_depth_cloud"
    p_in.header.stamp = this->now();
    p_in.point = det.bbox.center.position;

    try {
      // Transform into robot frame
      p_out = tf_buffer_->transform(p_in, "base_link", tf2::durationFromSec(0.2));

      // Keep detections in front (x>0) and within 0.7m radius
      float dist = std::sqrt(p_out.point.x * p_out.point.x + p_out.point.y * p_out.point.y);
      if (p_out.point.x > 0.0 && dist < 0.7) valid_points.push_back(p_out.point);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF2 transform failed: %s", ex.what());
    }
  }

  // STEP 4: No nearby obstacles then drive forward
  if (valid_points.empty()) {
    cmd.linear.x = 0.17;
    cmd.angular.z = 0.0;
    pub_cmd_vel_->publish(cmd);
    return;
  }

  // STEP 5: Get closest obstacle and steer based on its y position
  geometry_msgs::msg::Point closest = valid_points.front();
  float min_dist = std::sqrt(closest.x * closest.x + closest.y * closest.y);

  for (auto &p : valid_points) {
    float d = std::sqrt(p.x * p.x + p.y * p.y);
    if (d < min_dist) {
      min_dist = d;
      closest = p;
    }
  }

  if (closest.y > 0) { // obstacle to the left → turn right
    cmd.linear.x = 0.1;
    cmd.angular.z = -0.2;

  } else {             // obstacle to the right → turn left
    cmd.linear.x = 0.1;
    cmd.angular.z = 0.2;
  }

  pub_cmd_vel_->publish(cmd);
}
