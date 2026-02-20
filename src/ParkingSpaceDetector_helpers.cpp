#include "ParkingSpaceDetector.hpp"
#include <map>
#include <mutex>

// PointField helpers
// findFild searches for a field by name in the PointCloud2 message
bool ParkingSpaceDetector::findField(const sensor_msgs::msg::PointCloud2 &cloud,
                                     const std::string &name, FieldInfo &out)
{
  for (const auto &f : cloud.fields) {
    if (f.name == name && f.count >= 1) { out.offset = f.offset; out.datatype = f.datatype; return true; }
  }
  return false;
}

// this shouldn't be done and I hate it but it is the only way to read different datatypes from PointCloud2
float ParkingSpaceDetector::readAsFloat(const uint8_t *ptr, const FieldInfo &fi)
{
  switch (fi.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: { float v; std::memcpy(&v, ptr + fi.offset, sizeof(float)); return v; }
    case sensor_msgs::msg::PointField::UINT16:  { uint16_t v; std::memcpy(&v, ptr + fi.offset, sizeof(uint16_t)); return static_cast<float>(v); }
    case sensor_msgs::msg::PointField::INT16:   { int16_t  v; std::memcpy(&v, ptr + fi.offset, sizeof(int16_t));  return static_cast<float>(v); }
    case sensor_msgs::msg::PointField::UINT8:   { uint8_t  v = *(ptr + fi.offset); return static_cast<float>(v); }
    case sensor_msgs::msg::PointField::INT8:    { int8_t   v = *(reinterpret_cast<const int8_t*>(ptr + fi.offset)); return static_cast<float>(v); }
    default: return std::numeric_limits<float>::quiet_NaN();
  }
}

// assemble XYZI points
PointCloud::Ptr ParkingSpaceDetector::toPCLXYZI(const sensor_msgs::msg::PointCloud2 &cloud)
{
  FieldInfo fx, fy, fz, fi;
  bool has_x = findField(cloud, "x", fx);
  bool has_y = findField(cloud, "y", fy);
  bool has_z = findField(cloud, "z", fz);
  bool has_i = findField(cloud, "intensity", fi);
  if (!(has_x && has_y && has_z)) return PointCloud::Ptr(new PointCloud);

  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  const size_t step = cloud.point_step;
  const uint8_t *base = cloud.data.data();

  PointCloud::Ptr out(new PointCloud);
  out->points.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    const uint8_t *p = base + i*step;
    float x = readAsFloat(p, fx), y = readAsFloat(p, fy), z = readAsFloat(p, fz);
    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

    float inten = 0.0f;
    if (has_i) { inten = readAsFloat(p, fi); if (std::isnan(inten)) inten = 0.0f; }

    PointT pt; pt.x = x; pt.y = y; pt.z = z; pt.intensity = inten;
    out->points.push_back(pt);
  }
  out->width = out->points.size(); out->height = 1; out->is_dense = false;
  return out;
}

// 2D voxel grid: one representative point per (x,y) voxel cell.
// All output points share the cloud's mean Z so RANSAC works on a flat surface.
PointCloud::Ptr ParkingSpaceDetector::voxelFilter2D(const PointCloud::Ptr &cloud, float leaf_size)
{
  if (!cloud || cloud->empty() || leaf_size <= 0.0f) return cloud;

  struct Accum { float sx{0}, sy{0}; int n{0}; };
  std::map<std::pair<int32_t, int32_t>, Accum> cells;

  const float inv = 1.0f / leaf_size;
  float sum_z = 0.0f;

  for (const auto &pt : cloud->points) {
    auto key = std::make_pair(
      static_cast<int32_t>(std::floor(pt.x * inv)),
      static_cast<int32_t>(std::floor(pt.y * inv)));
    auto &a = cells[key];
    a.sx += pt.x; a.sy += pt.y; ++a.n;
    sum_z += pt.z;
  }

  const float mean_z = sum_z / static_cast<float>(cloud->size());

  PointCloud::Ptr out(new PointCloud);
  out->points.reserve(cells.size());
  for (const auto &[key, a] : cells) {
    PointT p;
    p.x = a.sx / a.n;
    p.y = a.sy / a.n;
    p.z = mean_z;
    p.intensity = 0.0f;
    out->points.push_back(p);
  }
  out->width = static_cast<uint32_t>(out->points.size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

// Check if a point is within the defined ROI
// this may be slower than a pcl bounding box check, but more flexible
bool ParkingSpaceDetector::inROI(const PointT& p) const {
  if (!use_roi_) return true;
  
  // Use bbox as ROI if enabled
  if (use_bbox_roi_) {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(bbox_mutex_));
    if (bbox_corners_lidar_.size() != 4) return false; // No bbox available yet
    
    // 2D point-in-polygon test
    // Check if point (p.x, p.y) is inside the bbox quadrilateral
    // TL->TR->BR->BL (indices 0,1,3,2)
    float x = p.x;
    float y = p.y;
    
    static const size_t order[4] = {0, 1, 3, 2};
    bool inside = false;
    for (size_t k = 0; k < 4; ++k) {
      size_t i = order[k], j = order[(k + 3) % 4];
      float xi = bbox_corners_lidar_[i].x();
      float yi = bbox_corners_lidar_[i].y();
      float xj = bbox_corners_lidar_[j].x();
      float yj = bbox_corners_lidar_[j].y();
      
      if (((yi > y) != (yj > y)) &&
          (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
        inside = !inside;
      }
    }
    
    return inside;
  }
  
  // Use fixed ROI bounds
  return (p.x >= roi_x_min_ && p.x <= roi_x_max_ &&
          p.y >= roi_y_min_ && p.y <= roi_y_max_ &&
          p.z >= roi_z_min_ && p.z <= roi_z_max_);
}
