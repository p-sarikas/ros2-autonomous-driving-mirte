/**
 * @file pcl_obstacle_detector.cpp
 * @brief Implements the 3D obstacle detection node for the Mirte robot.
 *
 * The file defines:
 *   1. One node class: PclObstacleDetector
 *   2. One callback function (class method - member function):
 *      - cloudCallback() — reacts to /mirte/camera_depth/points
 *
 * This node:
 *   - Subscribes to /mirte/camera_depth/points (depth camera point cloud)
 *   - Converts data to PCL format
 *   - Filters invalid or distant points
 *   - Removes the ground plane using RANSAC
 *   - Performs Euclidean clustering
 *   - Publishes detected clusters as 3D bounding boxes on /detections
 */

#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"
#include <pcl/common/common.h>
using std::placeholders::_1;


PclObstacleDetector::PclObstacleDetector()
: Node("pcl_obstacle_detector_node")
{
  // 1. Subscription to the Depth Camera
  // Queue size of subscriber is 1.
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/mirte/camera_depth/points", 1,
    std::bind(&PclObstacleDetector::cloudCallback, this, _1));

  pub_detections_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("/detections", 2);
}


// --- cloud Callback --- 
/*
Triggered by: A new point cloud message arriving on /mirte/camera_depth/points (published by Mirte’s simulated depth camera).
What it does:
  1. Conversion ROS → PCL
  2. Filtering
  3. Euclidean cluster extraction
  4. Build Detection3DArray
  5. Publish Detected Obstacles

Called whenever a new point cloud is received from the depth camera.
Converts the data to PCL, filters points, removes the ground plane,
segments obstacles, builds bounding boxes, and publishes them on /detections.
*/

void PclObstacleDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
  // 2. Conversion ROS → PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // 3.1. Filtering
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  // PassThrough Filter removes invalid or distant points (beyond 3 m)
  pass.setFilterLimits(0.0, 3.0); 
  pass.filter(*cloud);

  // 3.2 Remove ground plane
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Distance threshold to 0.01.
  seg.setDistanceThreshold(0.01); 
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coeff);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud);

  // 4. Euclidean cluster extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  // Distance between points in same cluster
  ec.setClusterTolerance(0.01);
  // Min cluster size: 5
  ec.setMinClusterSize(5);
  // Max cluster size: 25000
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // 5. Build Detection3DArray
  vision_msgs::msg::Detection3DArray detections_msg;
  detections_msg.header.stamp = msg->header.stamp;
  detections_msg.header.frame_id = "camera_depth_optical_frame";

  for (const auto& cluster : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto idx : cluster.indices)
      cluster_cloud->push_back(cloud->points[idx]);

    // Gets called every time the depth camera publishes a new point cloud on /mirte/camera_depth/points.
    pcl::PointXYZ min_pt, max_pt; 
    // getMinMax3D: calculate the extremes of the cluster, and use it to fill the bbox.size field of the message.
    pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);

    vision_msgs::msg::Detection3D det;
    det.header.frame_id = "mirte_depth_cloud";
    det.bbox.center.position.x = (min_pt.x + max_pt.x) / 2.0;
    det.bbox.center.position.y = (min_pt.y + max_pt.y) / 2.0;
    det.bbox.center.position.z = (min_pt.z + max_pt.z) / 2.0;
    det.bbox.size.x = max_pt.x - min_pt.x;
    det.bbox.size.y = max_pt.y - min_pt.y;
    det.bbox.size.z = max_pt.z - min_pt.z;

    // for debugging
    RCLCPP_INFO(rclcpp::get_logger("pcl_debug"),
    "Cluster center: x=%.2f y=%.2f z=%.2f", 
    det.bbox.center.position.x,
    det.bbox.center.position.y,
    det.bbox.center.position.z);
    // for debugging

    detections_msg.detections.push_back(det);
  }

  RCLCPP_INFO(this->get_logger(), "Detected %zu clusters", cluster_indices.size());
  last_msg_ = std::make_shared<vision_msgs::msg::Detection3DArray>(detections_msg);

  // 6. Publish Detected Obstacles
  pub_detections_->publish(detections_msg);
}
 



