#ifndef PARKING_SPACE_DETECTOR_HPP
#define PARKING_SPACE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

class ParkingSpaceDetector : public rclcpp::Node
{
public:
  ParkingSpaceDetector();

private:
  //  PointField helpers 
  struct FieldInfo { int32_t offset{-1}; uint8_t datatype{0}; };

  static bool findField(const sensor_msgs::msg::PointCloud2 &cloud,
                        const std::string &name, FieldInfo &out);

  static float readAsFloat(const uint8_t *ptr, const FieldInfo &fi);

  static PointCloud::Ptr toPCLXYZI(const sensor_msgs::msg::PointCloud2 &cloud);

  //  Core callback 
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Ambient filter
  static sensor_msgs::msg::PointCloud2
  filterByAmbient(const sensor_msgs::msg::PointCloud2 &in,
                  const FieldInfo &famb, double amin, double amax);

  //  Detection bits 
  struct Line {
    Eigen::Vector3f point;
    Eigen::Vector3f direction;
    std::vector<int> inliers;
    float length{0};
    Eigen::Vector3f start_point;
    Eigen::Vector3f end_point;
    std::vector<Eigen::Vector3f> inlier_pts; // for merging (3D, but we use XY)
  };

  struct ParkingSpace {
    Eigen::Vector3f center{0,0,0};
    Eigen::Vector3f corner1, corner2, corner3, corner4;
    float width{0}, length{0};
    float angle{0};
  };

  bool inROI(const PointT& p) const;

  std::vector<Line> detectLines(const PointCloud::Ptr &cloud);

  //  Merge co-linear lines (2D, PCA-based) 
  struct Line2D {
    Eigen::Vector2f p;   // point on line
    Eigen::Vector2f d;   // unit direction
    float z;             // avg z
    Eigen::Vector2f a;   // start
    Eigen::Vector2f b;   // end
    float span_min, span_max;
  };

  static Line2D make2D_from_Line(const Line& L);

  static float angleBetweenDeg(const Eigen::Vector2f& a, const Eigen::Vector2f& b);

  static float perpDist(const Line2D& A, const Line2D& B);

  static float overlapAlongA(const Line2D& A, const Line2D& B);

  std::vector<Line> mergeColinearLines2D(const std::vector<Line>& lines);

  //  U-shape detection (2D, robust) 
  static Line2D make2D(const Line& L);

  static float perpendicularDistance(const Line2D& A, const Line2D& B);

  static float projectedOverlap(const Line2D& A, const Line2D& B);

  static bool backAtLineEnds(const Line2D& A, const Line2D& B, const Line2D& C, float end_tol);

  std::vector<ParkingSpace> findParkingSpaces(const std::vector<Line> &lines);

  //  Visualization 
  void publishDetectedLines(const std::vector<Line> &lines, const std_msgs::msg::Header &hdr);

  void publishParkingMarkers(const std::vector<ParkingSpace> &spaces, const std_msgs::msg::Header &hdr);

  void clearAllMarkers(const std_msgs::msg::Header &hdr);

  //  Members 
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lines_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Filter params
  double ambient_min_{800.0}, ambient_max_{1300.0};
  bool   enabled_{true};
  std::string input_topic_{"ouster/points"};
  double intensity_min_{0.0}, intensity_max_{0.0};

  // ROI
  bool   use_roi_{true};
  double roi_x_min_{-8.0}, roi_x_max_{6.0};
  double roi_y_min_{-4.0}, roi_y_max_{4.0};
  double roi_z_min_{-2.0}, roi_z_max_{0.2};

  // RANSAC / line gates
  double line_threshold_{0.08};
  int    min_line_points_{24};
  double flat_z_abs_max_{0.15};
  double min_geom_length_{0.80};

  // Merge
  double merge_angle_tol_deg_{8.0};
  double merge_dist_tol_{0.18};
  double merge_min_overlap_{0.30};
  double min_merged_length_{1.0};

  // Parking geometry
  double parking_width_{2.0}, parking_length_{4.0};
  double width_tol_{0.35}, overlap_ratio_{0.60}, end_tol_{0.50};
};

#endif // PARKING_SPACE_DETECTOR_HPP
