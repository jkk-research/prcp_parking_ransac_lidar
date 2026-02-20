#include "ParkingSpaceDetector.hpp"
#include <random>
#include <unordered_set>
#include <limits>

std::vector<ParkingSpaceDetector::LineSegment>
ParkingSpaceDetector::detectLines(const PointCloud::Ptr &cloud,
                                  const std_msgs::msg::Header &header)
{
  std::vector<LineSegment> result;
  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker del;
  del.header = header;
  del.ns     = "detected_lines";
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  if (!ransac_enabled_ || !cloud || cloud->empty()) {
    detected_lines_pub_->publish(arr);
    return result;
  }

  std::vector<int> remaining;
  remaining.reserve(cloud->size());
  for (int i = 0; i < static_cast<int>(cloud->size()); ++i)
    remaining.push_back(i);

  std::mt19937 rng(42);
  int line_idx = 0;

  while (line_idx < ransac_max_lines_ &&
         static_cast<int>(remaining.size()) >= ransac_min_inliers_)
  {
    const int n = static_cast<int>(remaining.size());
    std::uniform_int_distribution<int> dist(0, n - 1);

    std::vector<int> best_inliers;

    for (int iter = 0; iter < ransac_max_iterations_; ++iter) {
      int a = dist(rng), b = dist(rng);
      if (a == b) continue;

      const PointT &pa = cloud->points[remaining[a]];
      const PointT &pb = cloud->points[remaining[b]];

      Eigen::Vector2f p0(pa.x, pa.y);
      Eigen::Vector2f dir(pb.x - pa.x, pb.y - pa.y);
      float len = dir.norm();

      if (len < static_cast<float>(ransac_min_sample_dist_)) continue;
      dir /= len;

      std::vector<int> inliers;
      const float thresh = static_cast<float>(ransac_line_dist_threshold_);
      for (int idx : remaining) {
        const PointT &q = cloud->points[idx];
        Eigen::Vector2f diff(q.x - p0.x(), q.y - p0.y());
        if (std::abs(diff.x() * dir.y() - diff.y() * dir.x()) <= thresh)
          inliers.push_back(idx);
      }

      if (static_cast<int>(inliers.size()) > static_cast<int>(best_inliers.size()))
        best_inliers = std::move(inliers);
    }

    if (static_cast<int>(best_inliers.size()) < ransac_min_inliers_)
      break; 

    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    float sum_z = 0.0f;
    for (int idx : best_inliers) {
      centroid += Eigen::Vector2f(cloud->points[idx].x, cloud->points[idx].y);
      sum_z    += cloud->points[idx].z;
    }
    centroid /= static_cast<float>(best_inliers.size());
    float mean_z = sum_z / static_cast<float>(best_inliers.size());

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (int idx : best_inliers) {
      Eigen::Vector2f d(cloud->points[idx].x - centroid.x(),
                        cloud->points[idx].y - centroid.y());
      cov += d * d.transpose();
    }
    Eigen::Vector2f line_dir(1.0f, 0.0f);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
    if (solver.info() == Eigen::Success)
      line_dir = solver.eigenvectors().col(1); // principal axis


    std::vector<std::pair<float, int>> proj_idx;
    proj_idx.reserve(best_inliers.size());
    for (int idx : best_inliers) {
      float p = (Eigen::Vector2f(cloud->points[idx].x, cloud->points[idx].y)
                 - centroid).dot(line_dir);
      proj_idx.emplace_back(p, idx);
    }
    std::sort(proj_idx.begin(), proj_idx.end());

    const float max_gap = static_cast<float>(ransac_max_line_gap_);
    int seg_s = 0;
    int best_s = 0, best_e = static_cast<int>(proj_idx.size()) - 1;
    float best_seg_len = -1.0f;

    for (int i = 1; i <= static_cast<int>(proj_idx.size()); ++i) {
      bool end_seg = (i == static_cast<int>(proj_idx.size())) ||
                     (proj_idx[i].first - proj_idx[i - 1].first > max_gap);
      if (end_seg) {
        float slen = proj_idx[i - 1].first - proj_idx[seg_s].first;
        if (slen > best_seg_len) {
          best_seg_len = slen;
          best_s = seg_s;
          best_e = i - 1;
        }
        seg_s = i;
      }
    }

    float min_proj = proj_idx[best_s].first;
    float max_proj = proj_idx[best_e].first;

    std::vector<int> segment_inliers;
    segment_inliers.reserve(best_e - best_s + 1);
    for (int i = best_s; i <= best_e; ++i)
      segment_inliers.push_back(proj_idx[i].second);

    // Always remove the segment's points from remaining so we don't
    // rediscover the same cluster, regardless of whether it passes validation.
    {
      std::unordered_set<int> seg_set(segment_inliers.begin(), segment_inliers.end());
      std::vector<int> next_r;
      next_r.reserve(remaining.size() - segment_inliers.size());
      for (int idx : remaining)
        if (!seg_set.count(idx)) next_r.push_back(idx);
      remaining = std::move(next_r);
    }

    if (static_cast<int>(segment_inliers.size()) < ransac_min_inliers_ ||
        (max_proj - min_proj) < static_cast<float>(ransac_min_line_length_))
      continue; // discard — don't count toward line_idx

    LineSegment seg;
    seg.centroid  = centroid;
    seg.dir       = line_dir;
    seg.min_proj  = min_proj;
    seg.max_proj  = max_proj;
    seg.mean_z    = mean_z;
    result.push_back(seg);

    Eigen::Vector2f start = centroid + min_proj * line_dir;
    Eigen::Vector2f end   = centroid + max_proj * line_dir;

    visualization_msgs::msg::Marker m;
    m.header   = header;
    m.ns       = "detected_lines";
    m.id       = line_idx;
    m.type     = visualization_msgs::msg::Marker::LINE_LIST;
    m.action   = visualization_msgs::msg::Marker::ADD;
    m.scale.x  = 0.05;
    m.color.r  = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point p1, p2;
    p1.x = start.x(); p1.y = start.y(); p1.z = mean_z;
    p2.x = end.x();   p2.y = end.y();   p2.z = mean_z;
    m.points.push_back(p1);
    m.points.push_back(p2);
    arr.markers.push_back(m);

    ++line_idx;
  }

  detected_lines_pub_->publish(arr);
  return result;
}
