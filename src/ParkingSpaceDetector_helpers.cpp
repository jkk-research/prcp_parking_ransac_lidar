#include "ParkingSpaceDetector.hpp"

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

// Check if a point is within the defined ROI
// this may be slower than a pcl bounding box check, but more flexible, and realistically it's so slow atm it doesn't matter much
bool ParkingSpaceDetector::inROI(const PointT& p) const {
  if (!use_roi_) return true;
  return (p.x >= roi_x_min_ && p.x <= roi_x_max_ &&
          p.y >= roi_y_min_ && p.y <= roi_y_max_ &&
          p.z >= roi_z_min_ && p.z <= roi_z_max_);
}
