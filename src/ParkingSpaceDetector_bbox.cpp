#include "ParkingSpaceDetector.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>

void ParkingSpaceDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (camera_intrinsics_.valid) return; // already set
  
  // Extract 3x3 intrinsic matrix from CameraInfo K array
  const auto& k = msg->k;
  camera_intrinsics_.mat << 
    k[0], k[1], k[2],
    k[3], k[4], k[5],
    k[6], k[7], k[8];
  
  camera_intrinsics_.mat_inv = camera_intrinsics_.mat.inverse();
  camera_intrinsics_.valid = true;
  
  RCLCPP_INFO(this->get_logger(), "Camera intrinsics received and stored");
}

Eigen::Vector3f ParkingSpaceDetector::deprojectPixel(int u, int v)
{
  // Deproject pixel to 3D point on ground plane (matches Python CameraModel)
  Eigen::Vector3f point_h(static_cast<float>(u), static_cast<float>(v), 1.0f);
  Eigen::Vector3f point_c = camera_intrinsics_.mat_inv * point_h;
  
  // Ground plane normal in camera frame: [0, 1, 0]
  Eigen::Vector3f norm_vec(0.0f, 1.0f, 0.0f);
  float denominator = norm_vec.dot(point_c);
  
  if (std::abs(denominator) < 1e-9f) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Deproject denominator near zero"); // this means the ray is parallel to ground plane
    return Eigen::Vector3f::Zero();
  }
  
  Eigen::Vector3f camera_coords = point_c / denominator;
  
  return Eigen::Vector3f(camera_coords[2], -camera_coords[0], camera_coords[1]);
}

void ParkingSpaceDetector::bboxCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (!msg) return;
  if (msg->data.size() < 4) return;

  // Skip empty/zero detections
  bool all_zero = true;
  for (size_t i = 0; i < 4; ++i) if (msg->data[i] != 0) { all_zero = false; break; }
  if (all_zero) return;

  // Check if camera intrinsics are ready
  if (!camera_intrinsics_.valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Camera intrinsics not yet received, skipping bbox");
    return;
  }

  // Parse bbox: x, y, w, h
  int x = msg->data[0];
  int y = msg->data[1];
  int w = msg->data[2];
  int h = msg->data[3];

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "BBox received: x=%d, y=%d, w=%d, h=%d", x, y, w, h);

  // Calculate 4 corner pixels
  std::vector<std::pair<int, int>> corners = {
    {x, y},         // top-left
    {x + w, y},     // top-right
    {x, y + h},     // bottom-left
    {x + w, y + h}  // bottom-right
  };

  // Deproject each corner to 3D in camera frame
  std::vector<Eigen::Vector3f> corners_cam;
  for (const auto& [u, v] : corners) {
    corners_cam.push_back(deprojectPixel(u, v));
  }

  // Transform all corners to lidar frame
  std::vector<Eigen::Vector3f> corners_lidar;
  try {
    auto transform = tf_buffer_->lookupTransform(
      lidar_frame_, camera_frame_, 
      rclcpp::Time(0), 
      rclcpp::Duration::from_seconds(0.1));

    for (const auto& pt_cam : corners_cam) {
      geometry_msgs::msg::PointStamped p_cam, p_lidar;
      p_cam.header.stamp = rclcpp::Time(0);
      p_cam.header.frame_id = camera_frame_;
      p_cam.point.x = pt_cam.x();
      p_cam.point.y = pt_cam.y();
      p_cam.point.z = pt_cam.z();

      tf2::doTransform(p_cam, p_lidar, transform);
      corners_lidar.emplace_back(p_lidar.point.x, p_lidar.point.y, p_lidar.point.z);
    }
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Failed to transform bbox from %s to %s: %s",
                         camera_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    return;
  }

  // Scale corners around center if bbox_roi_scale != 1.0
  std::vector<Eigen::Vector3f> scaled_corners = corners_lidar;
  if (bbox_roi_scale_ != 1.0f) {
    // Compute center of the 4 corners
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for (const auto& c : corners_lidar) center += c;
    center /= static_cast<float>(corners_lidar.size());
    
    // Scale each corner around the center
    for (auto& corner : scaled_corners) {
      Eigen::Vector3f vec = corner - center;
      corner = center + vec * static_cast<float>(bbox_roi_scale_);
    }
  }
  
  // Store scaled corners for ROI usage
  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    bbox_corners_lidar_ = scaled_corners;
  }
  
  if (use_bbox_roi_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "BBox ROI updated with %zu corners (scale: %.2f)", 
                         scaled_corners.size(), bbox_roi_scale_);
  }

  // Create LINE_STRIP marker showing the rectangle
  visualization_msgs::msg::Marker m;
  m.header.stamp = this->now();
  m.header.frame_id = lidar_frame_;
  m.ns = "camera_bboxes";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.1; // line width
  m.color.r = 0.0f; m.color.g = 0.6f; m.color.b = 1.0f; m.color.a = 0.9f;
  m.pose.orientation.w = 1.0;
  // press to 2d
  for (auto& pt : scaled_corners) { pt.z() = 0.0f; }

  // Add corners in order to form a rectangle: TL -> TR -> BR -> BL -> TL
  for (size_t idx : {0, 1, 3, 2, 0}) {
    geometry_msgs::msg::Point p;
    p.x = scaled_corners[idx].x();
    p.y = scaled_corners[idx].y();
    p.z = scaled_corners[idx].z() + 0.05; // slight offset for visibilitys
    m.points.push_back(p);
  }

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(m);
  bbox_marker_pub_->publish(arr);
}
