#include "ParkingSpaceDetector.hpp"

// callback
void ParkingSpaceDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) return;

  sensor_msgs::msg::PointCloud2 filtered = *msg;
  bool used_filter = false;
  if (enabled_) {
    FieldInfo famb;
    if (findField(*msg, "ambient", famb)) { used_filter = true; filtered = filterByAmbient(*msg, famb, ambient_min_, ambient_max_); }
    else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No 'ambient' field found; running detection on unfiltered cloud.");
    }
  }
  filtered_pub_->publish(filtered);

  PointCloud::Ptr pcl_cloud = toPCLXYZI(filtered);
  if (pcl_cloud->empty()) { clearAllMarkers(filtered.header); return; }

  auto lines  = detectLines(pcl_cloud);              // includes ROI + merge
  publishDetectedLines(lines, filtered.header);

  auto spaces = findParkingSpaces(lines);
  publishParkingMarkers(spaces, filtered.header);

  // Publish PoseArray for all spaces
  geometry_msgs::msg::PoseArray pa;
  pa.header = filtered.header;               // frame_id stays the same as the cloud
  pa.poses.reserve(spaces.size());
  for (const auto &s : spaces) pa.poses.push_back(poseFromSpace(s));
  poses_pub_->publish(pa);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Lines: %zu, Parking spaces: %zu", lines.size(), spaces.size());
}

// Ambient filter
sensor_msgs::msg::PointCloud2
ParkingSpaceDetector::filterByAmbient(const sensor_msgs::msg::PointCloud2 &in,
                                      const FieldInfo &famb, double amin, double amax)
{
  const size_t n = static_cast<size_t>(in.width) * in.height;
  const size_t step = in.point_step;
  const uint8_t *base = in.data.data();

  sensor_msgs::msg::PointCloud2 out;
  out.header = in.header; out.height = 1; out.width = 0;
  out.fields = in.fields; out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step; out.is_dense = false;
  out.data.reserve(in.data.size());

  size_t kept = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t *src = base + i*step;
    float a = readAsFloat(src, famb);
    if (std::isnan(a)) continue;
    if (a >= static_cast<float>(amin) && a <= static_cast<float>(amax)) {
      const size_t start = out.data.size();
      out.data.resize(start + step);
      std::memcpy(out.data.data() + start, src, step);
      ++kept;
    }
  }
  out.width = static_cast<uint32_t>(kept);
  out.row_step = out.point_step * out.width;
  return out;
}
