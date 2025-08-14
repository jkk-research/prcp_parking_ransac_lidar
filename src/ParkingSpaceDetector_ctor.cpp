#include "ParkingSpaceDetector.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

// Constructor
ParkingSpaceDetector::ParkingSpaceDetector()
: rclcpp::Node("prcp_parking_ransac_lidar_node")
{
  //  embient filter parameters
  ambient_min_   = declare_parameter<double>("ambient_min", 800.0); // minimum ambient value for filter
  ambient_max_   = declare_parameter<double>("ambient_max", 1300.0); // maximum ambient value for filter
  enabled_       = declare_parameter<bool>("enabled", true); // enable/disable ambient filter
  input_topic_   = declare_parameter<std::string>("input_topic", "ouster/points"); // input topic for point cloud
  intensity_min_ = declare_parameter<double>("intensity_min", 0.0); // 0 disables -- minimum intensity for points to keep
  intensity_max_ = declare_parameter<double>("intensity_max", 0.0); // 0 disables -- maximum intensity for points to keep

  //  ROI (to kill far clutter) 
  use_roi_ = declare_parameter<bool>("use_roi", true); // enable/disable ROI filtering
  roi_x_min_ = declare_parameter<double>("roi_x_min", -8.0); // minimum X coordinate in ROI
  roi_x_max_ = declare_parameter<double>("roi_x_max",  8.0); // etc you get it
  roi_y_min_ = declare_parameter<double>("roi_y_min", -8.0);
  roi_y_max_ = declare_parameter<double>("roi_y_max",  8.0);
  roi_z_min_ = declare_parameter<double>("roi_z_min", -20.0);
  roi_z_max_ = declare_parameter<double>("roi_z_max",  30.2);

  //  RANSAC / detection 
  line_threshold_   = declare_parameter<double>("line_threshold", 0.08); // meters, distance threshold for line fitting
  min_line_points_  = declare_parameter<int>("min_line_points", 24); // minimum number of points for a line to be valid

  // line acceptance gates
  flat_z_abs_max_   = declare_parameter<double>("flat_z_abs_max", 0.15);  // the maximum absolute Z deviation for a line to be considered flat
  min_geom_length_  = declare_parameter<double>("min_geom_length", 0.80); // minimum length of a line to be considered valid

  //  Merge colinear fragments (in case they didn't fit in the distance threshold in ransac)
  merge_angle_tol_deg_ = declare_parameter<double>("merge_angle_tol_deg", 8.0); // degrees, angle tolerance for merging lines
  merge_dist_tol_      = declare_parameter<double>("merge_dist_tol", 0.18); // m, perpendicular separation
  merge_min_overlap_   = declare_parameter<double>("merge_min_overlap", 0.30); // m, overlap before merging
  min_merged_length_   = declare_parameter<double>("min_merged_length", 1.0);  // m, minimum length of merged lines

  //  Parking geometry 
  parking_width_    = declare_parameter<double>("parking_width", 2.0); // m, width of parking space
  parking_length_   = declare_parameter<double>("parking_length", 4.0); // m, length of parking space
  width_tol_        = declare_parameter<double>("width_tol", 0.35);       // m, tolerance for width
  overlap_ratio_    = declare_parameter<double>("overlap_ratio", 0.60);   // fraction of length
  end_tol_          = declare_parameter<double>("end_tol", 0.50);         // m, tolerance for back at line ends

  // Live parameter updates
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params)
    {
      for (const auto &p : params) {
        const auto& n = p.get_name();
        if (n=="ambient_min") ambient_min_ = p.as_double();
        else if (n=="ambient_max") ambient_max_ = p.as_double();
        else if (n=="enabled") enabled_ = p.as_bool();
        else if (n=="intensity_min") intensity_min_ = p.as_double();
        else if (n=="intensity_max") intensity_max_ = p.as_double();

        else if (n=="use_roi") use_roi_ = p.as_bool();
        else if (n=="roi_x_min") roi_x_min_ = p.as_double();
        else if (n=="roi_x_max") roi_x_max_ = p.as_double();
        else if (n=="roi_y_min") roi_y_min_ = p.as_double();
        else if (n=="roi_y_max") roi_y_max_ = p.as_double();
        else if (n=="roi_z_min") roi_z_min_ = p.as_double();
        else if (n=="roi_z_max") roi_z_max_ = p.as_double();

        else if (n=="line_threshold") line_threshold_ = p.as_double();
        else if (n=="min_line_points") min_line_points_ = p.as_int();
        else if (n=="flat_z_abs_max")  flat_z_abs_max_ = p.as_double();
        else if (n=="min_geom_length") min_geom_length_ = p.as_double();

        else if (n=="merge_angle_tol_deg") merge_angle_tol_deg_ = p.as_double();
        else if (n=="merge_dist_tol")      merge_dist_tol_      = p.as_double();
        else if (n=="merge_min_overlap")   merge_min_overlap_   = p.as_double();
        else if (n=="min_merged_length")   min_merged_length_   = p.as_double();

        else if (n=="parking_width")  parking_width_  = p.as_double();
        else if (n=="parking_length") parking_length_ = p.as_double();
        else if (n=="width_tol")      width_tol_      = p.as_double();
        else if (n=="overlap_ratio")  overlap_ratio_  = p.as_double();
        else if (n=="end_tol")        end_tol_        = p.as_double();
      }
      rcl_interfaces::msg::SetParametersResult res; res.successful = true; return res;
    });

  // Pub/Sub
  using rclcpp::SensorDataQoS;
  filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10); // just to validate if we can still see the parking space with the filter on
  marker_pub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>("detected_parking_spaces", 10); // visualization of detected parking spaces
  lines_pub_    = this->create_publisher<visualization_msgs::msg::MarkerArray>("detected_lines", 10); // visualization of detected lines

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, SensorDataQoS(),
    std::bind(&ParkingSpaceDetector::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Parking Space Detector runningasd (input: %s)", input_topic_.c_str());
}
