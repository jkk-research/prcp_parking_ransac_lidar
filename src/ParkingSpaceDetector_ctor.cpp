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

  // Live parameter updates (only for parameters we keep)
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
      }
      rcl_interfaces::msg::SetParametersResult res; res.successful = true; return res;
    });

  // Pub/Sub
  using rclcpp::SensorDataQoS;
  filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

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
