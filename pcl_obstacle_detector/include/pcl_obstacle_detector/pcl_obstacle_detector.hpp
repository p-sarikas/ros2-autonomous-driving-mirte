/**
 * @file pcl_obstacle_detector.hpp
 * @brief ROS2 Node for detecting 3D obstacles using PCL.
 * 
 * Subscribes to the depth camera point cloud (/mirte/camera_depth/points),
 * segments the environment using PCL, and publishes detected clusters
 * as 3D bounding boxes on the /detections topic.
 */


#ifndef PCL_OBSTACLE_DETECTOR_HPP_
#define PCL_OBSTACLE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


/** Subscribes to /mirte/camera_depth/points and publishes 3D obstacle detections
    as vision_msgs::Detection3DArray messages on the /detections topic.  */
class PclObstacleDetector : public rclcpp::Node
{
public:
  /** Constructor – initializes subscribers and publishers. */
  PclObstacleDetector();

private:
  /** Callback function that processes the incoming point cloud. */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_detections_;
  rclcpp::TimerBase::SharedPtr timer_;  // timer for republishing
  vision_msgs::msg::Detection3DArray::SharedPtr last_msg_;  // stores last published detections

};

#endif  // PCL_OBSTACLE_DETECTOR_HPP_








