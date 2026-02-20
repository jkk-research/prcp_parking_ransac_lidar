#include "ParkingSpaceDetector.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
// Constructor
ParkingSpaceDetector::ParkingSpaceDetector()
: rclcpp::Node("prcp_parking_ransac_lidar_node")
{
  // Ambient filter parameters
  ambient_min_   = declare_parameter<double>("ambient_min", 800.0);
  ambient_max_   = declare_parameter<double>("ambient_max", 1100.0);
  enabled_       = declare_parameter<bool>("enabled", true);
  input_topic_   = declare_parameter<std::string>("input_topic", "ouster/points");

  // ROI
  use_roi_ = declare_parameter<bool>("use_roi", true);
  roi_x_min_ = declare_parameter<double>("roi_x_min", -10.0);
  roi_x_max_ = declare_parameter<double>("roi_x_max",  10.0);
  roi_y_min_ = declare_parameter<double>("roi_y_min", -10.0);
  roi_y_max_ = declare_parameter<double>("roi_y_max",  10.0);
  roi_z_min_ = declare_parameter<double>("roi_z_min", -20.0);
  roi_z_max_ = declare_parameter<double>("roi_z_max",  30.2);

  // BBox ROI
  use_bbox_roi_ = declare_parameter<bool>("use_bbox_roi", true);
  bbox_roi_scale_ = declare_parameter<double>("bbox_roi_scale", 2.0);

  // Live parameter updates
  param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params)
    {
      for (const auto &p : params) {
        const auto& n = p.get_name();
        if (n=="ambient_min") ambient_min_ = p.as_double();
        else if (n=="ambient_max") ambient_max_ = p.as_double();
        else if (n=="enabled") enabled_ = p.as_bool();

        else if (n=="use_roi") use_roi_ = p.as_bool();
        else if (n=="roi_x_min") roi_x_min_ = p.as_double();
        else if (n=="roi_x_max") roi_x_max_ = p.as_double();
        else if (n=="roi_y_min") roi_y_min_ = p.as_double();
        else if (n=="roi_y_max") roi_y_max_ = p.as_double();
        else if (n=="roi_z_min") roi_z_min_ = p.as_double();
        else if (n=="roi_z_max") roi_z_max_ = p.as_double();
        else if (n=="use_bbox_roi") use_bbox_roi_ = p.as_bool();
        else if (n=="bbox_roi_scale") bbox_roi_scale_ = p.as_double();

        else if (n=="voxel_filter_enabled") voxel_filter_enabled_ = p.as_bool();
        else if (n=="voxel_leaf_size")      voxel_leaf_size_      = p.as_double();

        else if (n=="ransac_enabled")             ransac_enabled_             = p.as_bool();
        else if (n=="ransac_max_iterations")      ransac_max_iterations_      = static_cast<int>(p.as_int());
        else if (n=="ransac_line_dist_threshold") ransac_line_dist_threshold_ = p.as_double();
        else if (n=="ransac_min_inliers")         ransac_min_inliers_         = static_cast<int>(p.as_int());
        else if (n=="ransac_max_lines")           ransac_max_lines_           = static_cast<int>(p.as_int());
        else if (n=="ransac_min_line_length")     ransac_min_line_length_     = p.as_double();
        else if (n=="ransac_min_sample_dist")     ransac_min_sample_dist_     = p.as_double();
        else if (n=="ransac_max_line_gap")        ransac_max_line_gap_        = p.as_double();

        else if (n=="parking_detection_enabled") parking_detection_enabled_ = p.as_bool();
        else if (n=="parking_angle_tolerance")   parking_angle_tolerance_   = p.as_double();
        else if (n=="parking_width_min")         parking_width_min_         = p.as_double();
        else if (n=="parking_width_max")         parking_width_max_         = p.as_double();
        else if (n=="parking_length_min")        parking_length_min_        = p.as_double();
        else if (n=="parking_length_max")        parking_length_max_        = p.as_double();
        else if (n=="parking_line_length_min")   parking_line_length_min_   = p.as_double();
      }
      rcl_interfaces::msg::SetParametersResult res; res.successful = true; return res;
    });

  voxel_filter_enabled_ = declare_parameter<bool>("voxel_filter_enabled", false);
  voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.30);

  ransac_enabled_ = declare_parameter<bool>("ransac_enabled", true);

  ransac_max_iterations_ = declare_parameter<int>("ransac_max_iterations", 200);

  ransac_line_dist_threshold_ = declare_parameter<double>("ransac_line_dist_threshold", 0.15);

  ransac_min_inliers_ = declare_parameter<int>("ransac_min_inliers", 5);

  ransac_max_lines_ = declare_parameter<int>("ransac_max_lines", 8);

  ransac_min_line_length_ = declare_parameter<double>("ransac_min_line_length", 1.0);

  ransac_min_sample_dist_ = declare_parameter<double>("ransac_min_sample_dist", 0.3);


  ransac_max_line_gap_ = declare_parameter<double>("ransac_max_line_gap", 0.5);

  parking_detection_enabled_ = declare_parameter<bool>("parking_detection_enabled", true);


  parking_angle_tolerance_ = declare_parameter<double>("parking_angle_tolerance", 15.0);

  parking_width_min_ = declare_parameter<double>("parking_width_min", 1.5);
  parking_width_max_ = declare_parameter<double>("parking_width_max", 2.8);

  parking_length_min_ = declare_parameter<double>("parking_length_min", 2.0);
  parking_length_max_ = declare_parameter<double>("parking_length_max", 6.0);


  parking_line_length_min_ = declare_parameter<double>("parking_line_length_min", 1.5);

  // Pub/Sub
  using rclcpp::SensorDataQoS;
  filtered_pub_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
  detected_lines_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("detected_lines", 10);
  parking_spaces_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("parking_spaces", 10);

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, SensorDataQoS(),
    std::bind(&ParkingSpaceDetector::cloudCallback, this, std::placeholders::_1));

  // Bounding box / camera -> lidar visualization parameters
  bbox_topic_ = declare_parameter<std::string>("bbox_topic", "parking_places/detections/boxes");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "zed_camera_front");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "os_sensor");

  bbox_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    bbox_topic_, 10, std::bind(&ParkingSpaceDetector::bboxCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "zed2i/zed_node/left/camera_info", 10,
    std::bind(&ParkingSpaceDetector::cameraInfoCallback, this, std::placeholders::_1));

  bbox_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("camera_bbox", 10);

  // TF2 buffer/listener for transforming camera points to lidar frame
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Parking Space Detector running (input: %s)", input_topic_.c_str());
}
