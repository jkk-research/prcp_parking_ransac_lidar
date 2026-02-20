#include "ParkingSpaceDetector.hpp"
// callback: apply ambient filter and publish filtered cloud
void ParkingSpaceDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) return;

  sensor_msgs::msg::PointCloud2 filtered = *msg;
  if (enabled_) {
    if (use_bbox_roi_) {
      filtered = filterByBboxROI(filtered);
    }
    FieldInfo famb;
    if (findField(filtered, "ambient", famb)) {
      filtered = filterByAmbient(filtered, famb, ambient_min_, ambient_max_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No 'ambient' field found; publishing unfiltered cloud.");
    }
  }

  filtered_pub_->publish(filtered);

  // If cloud is empty after filter, clear markers
  PointCloud::Ptr pcl_cloud = toPCLXYZI(filtered);
  if (!pcl_cloud || pcl_cloud->empty()) { clearAllMarkers(filtered.header); }
}

// BBox ROI filter: keep only points inside the transformed+scaled bbox polygon
sensor_msgs::msg::PointCloud2
ParkingSpaceDetector::filterByBboxROI(const sensor_msgs::msg::PointCloud2 &in)
{
  FieldInfo fx, fy;
  if (!findField(in, "x", fx) || !findField(in, "y", fy)) return in;

  std::vector<Eigen::Vector3f> corners;
  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    if (bbox_corners_lidar_.size() != 4) return in;
    corners = bbox_corners_lidar_;
  }

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
    const uint8_t *src = base + i * step;
    float x = readAsFloat(src, fx);
    float y = readAsFloat(src, fy);
    if (std::isnan(x) || std::isnan(y)) continue;

    // 2D point-in-polygon (ray casting), same winding as visualization: TL->TR->BR->BL
    static const size_t order[4] = {0, 1, 3, 2};
    bool inside = false;
    for (size_t k = 0; k < 4; ++k) {
      size_t ci = order[k], cj = order[(k + 3) % 4];
      float xi = corners[ci].x(), yi = corners[ci].y();
      float xj = corners[cj].x(), yj = corners[cj].y();
      if (((yi > y) != (yj > y)) &&
          (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
        inside = !inside;
      }
    }
    if (!inside) continue;

    const size_t start = out.data.size();
    out.data.resize(start + step);
    std::memcpy(out.data.data() + start, src, step);
    ++kept;
  }
  out.width = static_cast<uint32_t>(kept);
  out.row_step = out.point_step * out.width;
  return out;
}

// Ambient filter implementation
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
