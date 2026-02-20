#ifndef PARKING_SPACE_DETECTOR_HPP
#define PARKING_SPACE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

class ParkingSpaceDetector : public rclcpp::Node
{
public:
  ParkingSpaceDetector();

private:
  // PointField helpers
  struct FieldInfo { int32_t offset{-1}; uint8_t datatype{0}; };

  static bool findField(const sensor_msgs::msg::PointCloud2 &cloud,
                        const std::string &name, FieldInfo &out);

  static float readAsFloat(const uint8_t *ptr, const FieldInfo &fi);

  static PointCloud::Ptr toPCLXYZI(const sensor_msgs::msg::PointCloud2 &cloud);

  bool inROI(const PointT& p) const;

  // Core callback: read cloud, apply ambient filter, publish filtered cloud
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Ambient filter
  static sensor_msgs::msg::PointCloud2
  filterByAmbient(const sensor_msgs::msg::PointCloud2 &in,
                  const FieldInfo &famb, double amin, double amax);

  // BBox ROI filter (applied before ambient filter)
  sensor_msgs::msg::PointCloud2
  filterByBboxROI(const sensor_msgs::msg::PointCloud2 &in);

  // BBox callbacks
  void bboxCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Camera model for deprojection
  struct CameraIntrinsics {
    Eigen::Matrix3f mat;
    Eigen::Matrix3f mat_inv;
    bool valid{false};
  };
  CameraIntrinsics camera_intrinsics_;

  // Deproject pixel (u,v) to 3D point on ground plane
  Eigen::Vector3f deprojectPixel(int u, int v);

  // Visualization helper to clear markers
  void clearAllMarkers(const std_msgs::msg::Header &hdr);

  // 2D voxel grid filter: collapses points with the same (x,y) voxel into one
  // centroid and flattens all output points to the cloud's mean Z.
  static PointCloud::Ptr voxelFilter2D(const PointCloud::Ptr &cloud, float leaf_size);

  // A single detected line segment (result of one RANSAC round + PCA refit).
  struct LineSegment {
    Eigen::Vector2f centroid;       // centroid of the inlier set on the XY plane
    Eigen::Vector2f dir;            // unit direction vector of the line
    float min_proj{0}, max_proj{0}; // signed extent from centroid along dir
    float mean_z{0};
    float length()               const { return max_proj - min_proj; }
    Eigen::Vector2f start()      const { return centroid + min_proj * dir; }
    Eigen::Vector2f end()        const { return centroid + max_proj * dir; }
  };

  // RANSAC line detection — returns the detected segments and publishes markers.
  std::vector<LineSegment> detectLines(const PointCloud::Ptr &cloud,
                                       const std_msgs::msg::Header &header);

  // Parking space detection: finds parallel line pairs matching the expected
  // parking space dimensions and publishes their outlines + labels.
  void findParkingSpaces(const std::vector<LineSegment> &lines,
                         const std_msgs::msg::Header &header);

  // Members
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detected_lines_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr parking_spaces_pub_;
  // bounding-box visualization
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_marker_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string bbox_topic_{"parking_places/detections/boxes"};
  std::string camera_frame_{"zed_camera_front"};
  std::string lidar_frame_{"os_sensor"};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Filter params
  double ambient_min_{800.0}, ambient_max_{1300.0};
  bool   enabled_{true};
  std::string input_topic_{"ouster/points"};

  // ROI
  bool   use_roi_{true};
  double roi_x_min_{-8.0}, roi_x_max_{6.0};
  double roi_y_min_{-4.0}, roi_y_max_{4.0};
  double roi_z_min_{-2.0}, roi_z_max_{0.2};

  // BBox ROI
  bool use_bbox_roi_{false};
  double bbox_roi_scale_{1.0}; // multiplier to inflate bbox ROI
  std::vector<Eigen::Vector3f> bbox_corners_lidar_; // 4 corners in lidar frame
  std::mutex bbox_mutex_;

  // 2D voxel grid params
  bool   voxel_filter_enabled_{true};
  double voxel_leaf_size_{0.30};

  // RANSAC line detection params
  bool   ransac_enabled_{true};
  int    ransac_max_iterations_{200};
  double ransac_line_dist_threshold_{0.15};
  int    ransac_min_inliers_{5};
  int    ransac_max_lines_{8};
  double ransac_min_line_length_{1.0};
  double ransac_min_sample_dist_{0.3};
  double ransac_max_line_gap_{0.5};

  // Parking space detection params
  bool   parking_detection_enabled_{true};
  double parking_angle_tolerance_{15.0}; // degrees — how far from parallel two lines may be
  double parking_width_min_{1.5};        // metres — minimum distance between the two lines
  double parking_width_max_{2.8};        // metres — maximum distance between the two lines
  double parking_length_min_{2.0};       // metres — minimum longitudinal overlap of the pair
  double parking_length_max_{6.0};       // metres — maximum longitudinal overlap of the pair
  double parking_line_length_min_{1.5};  // metres — individual line must be at least this long
};

#endif // PARKING_SPACE_DETECTOR_HPP
