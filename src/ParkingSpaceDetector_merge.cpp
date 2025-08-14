#include "ParkingSpaceDetector.hpp"


// this function converts a 3D line to 2D 
ParkingSpaceDetector::Line2D ParkingSpaceDetector::make2D_from_Line(const Line& L) {
  Line2D o;
  o.p = {L.point.x(), L.point.y()};
  o.d = Eigen::Vector2f(L.direction.x(), L.direction.y()).normalized();
  o.z = (L.point.z() + L.start_point.z() + L.end_point.z()) / 3.0f;
  o.a = {L.start_point.x(), L.start_point.y()};
  o.b = {L.end_point.x(),   L.end_point.y()};
  o.span_min = (o.a - o.p).dot(o.d);
  o.span_max = (o.b - o.p).dot(o.d);
  if (o.span_min > o.span_max) { std::swap(o.span_min, o.span_max); std::swap(o.a, o.b); }
  return o;
}

// calculate the angle between two 2D vectors in degrees
float ParkingSpaceDetector::angleBetweenDeg(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
  float c = std::clamp(a.normalized().dot(b.normalized()), -1.0f, 1.0f);
  return std::abs(std::atan2(std::sqrt(1.0f - c*c), c) * 180.0f / static_cast<float>(M_PI));
}

// calculate the perpendicular distance between two 2D lines
float ParkingSpaceDetector::perpDist(const Line2D& A, const Line2D& B) {
  Eigen::Vector2f n(-A.d.y(), A.d.x());
  return std::abs((B.p - A.p).dot(n));
}

// calculate the overlap of two 2D lines along the direction of line A
float ParkingSpaceDetector::overlapAlongA(const Line2D& A, const Line2D& B) {
  float bmin = (B.a - A.p).dot(A.d);
  float bmax = (B.b - A.p).dot(A.d);
  if (bmin > bmax) std::swap(bmin, bmax);
  float lo = std::max(A.span_min, bmin);
  float hi = std::min(A.span_max, bmax);
  return std::max(0.0f, hi - lo);
}

// merge colinear lines in 2D
std::vector<ParkingSpaceDetector::Line> ParkingSpaceDetector::mergeColinearLines2D(const std::vector<Line>& lines)
{
  const float ang_tol = static_cast<float>(merge_angle_tol_deg_);
  const float dist_tol = static_cast<float>(merge_dist_tol_);
  const float min_overlap = static_cast<float>(merge_min_overlap_);

  const size_t n = lines.size();
  std::vector<bool> used(n, false);
  std::vector<Line> merged;

  for (size_t i = 0; i < n; ++i) {
    if (used[i]) continue;
    Line2D Li = make2D_from_Line(lines[i]);

    // collect a cluster
    std::vector<size_t> cluster{ i };
    used[i] = true;
    for (size_t j = i+1; j < n; ++j) {
      if (used[j]) continue;
      Line2D Lj = make2D_from_Line(lines[j]);
      if (angleBetweenDeg(Li.d, Lj.d) <= ang_tol &&
          perpDist(Li, Lj) <= dist_tol &&
          (overlapAlongA(Li, Lj) > min_overlap || overlapAlongA(Lj, Li) > min_overlap)) {
        used[j] = true;
        cluster.push_back(j);
      }
    }

    // gather all inlier points from the cluster
    std::vector<Eigen::Vector2f> pts2;
    pts2.reserve(1000);
    float zsum = 0.0f; size_t zcnt = 0;
    for (size_t idx : cluster) {
      for (const auto& p3 : lines[idx].inlier_pts) {
        pts2.emplace_back(p3.x(), p3.y());
        zsum += p3.z(); ++zcnt;
      }
    }
    if (pts2.size() < 2) continue;

    // PCA fit in 2D (so we can find the principal axis)
    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    for (const auto& q : pts2) mean += q;
    mean /= static_cast<float>(pts2.size());

    Eigen::Matrix2f C = Eigen::Matrix2f::Zero();
    for (const auto& q : pts2) {
      Eigen::Vector2f d = q - mean;
      C += d * d.transpose();
    }
    C /= std::max<size_t>(1, pts2.size()-1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(C);
    Eigen::Vector2f dir = es.eigenvectors().col(1).normalized(); // principal axis

    // span along dir
    float tmin = std::numeric_limits<float>::max();
    float tmax = std::numeric_limits<float>::lowest();
    for (const auto& q : pts2) {
      float t = (q - mean).dot(dir);
      tmin = std::min(tmin, t);
      tmax = std::max(tmax, t);
    }
    Eigen::Vector2f A = mean + tmin * dir;
    Eigen::Vector2f B = mean + tmax * dir;
    float len = (B - A).norm();

    if (len < static_cast<float>(min_merged_length_)) continue;

    // build merged 3D line
    float gz = (zcnt>0) ? (zsum/static_cast<float>(zcnt)) : 0.0f;

    Line Lm;
    Lm.point = Eigen::Vector3f(mean.x(), mean.y(), gz);
    Lm.direction = Eigen::Vector3f(dir.x(), dir.y(), 0.0f).normalized();
    Lm.start_point = Eigen::Vector3f(A.x(), A.y(), gz);
    Lm.end_point   = Eigen::Vector3f(B.x(), B.y(), gz);
    Lm.length = len;
    merged.push_back(Lm);
  }

  return merged;
}
