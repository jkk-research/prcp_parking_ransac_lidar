#include "ParkingSpaceDetector.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

std::vector<ParkingSpaceDetector::Line> ParkingSpaceDetector::detectLines(const PointCloud::Ptr &cloud)
{
  // Intensity + ROI gate
  PointCloud::Ptr hi(new PointCloud);
  hi->points.reserve(cloud->points.size());
  for (const auto &p : cloud->points) {
    if (!inROI(p)) continue;
    if ((intensity_min_ <= 0 || p.intensity >= static_cast<float>(intensity_min_)) &&
        (intensity_max_ <= 0 || p.intensity <= static_cast<float>(intensity_max_))) {
      hi->points.push_back(p);
    }
  }
  if (hi->points.size() < static_cast<size_t>(min_line_points_)) {
    RCLCPP_WARN(this->get_logger(), "Too few points after ROI/intensity filter (%zu)", hi->points.size());
    return {};
  }

  PointCloud::Ptr remaining(new PointCloud(*hi));

  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(line_threshold_);
  seg.setMaxIterations(1000);

  std::vector<Line> raw_lines;
  const int max_lines = 20;

  while (remaining->points.size() > static_cast<size_t>(min_line_points_) && // if there are enough points
         static_cast<int>(raw_lines.size()) < max_lines)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients); // I have no clue how to use this properly, but it works (thx chatgpt)
    seg.setInputCloud(remaining);
    seg.segment(*inliers, *coeffs);
    if (inliers->indices.size() < static_cast<size_t>(min_line_points_)) break;

    Line line;
    line.point = Eigen::Vector3f(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
    line.direction = Eigen::Vector3f(coeffs->values[3], coeffs->values[4], coeffs->values[5]).normalized();

    float min_t = std::numeric_limits<float>::max();
    float max_t = std::numeric_limits<float>::lowest();
    line.inlier_pts.reserve(inliers->indices.size());
    for (int idx : inliers->indices) {
      const auto &pt = remaining->points[idx];
      Eigen::Vector3f p(pt.x, pt.y, pt.z);
      float t = (p - line.point).dot(line.direction);
      min_t = std::min(min_t, t);
      max_t = std::max(max_t, t);
      line.inlier_pts.push_back(p);
    }
    line.length = max_t - min_t;
    line.start_point = line.point + min_t * line.direction;
    line.end_point   = line.point + max_t * line.direction;
    line.inliers     = inliers->indices;

    // Early gates
    const bool flat_enough = std::abs(line.direction.z()) < flat_z_abs_max_;
    const bool long_enough = line.length >= static_cast<float>(min_geom_length_);
    if (flat_enough && long_enough) raw_lines.push_back(line);

    // remove inliers and continue
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.setNegative(true);
    PointCloud tmp;
    extract.filter(tmp);
    *remaining = tmp;
  }

  // Merge co-linear fragments
  return mergeColinearLines2D(raw_lines);
}
