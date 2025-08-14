#include "ParkingSpaceDetector.hpp"

// U-shape detection
ParkingSpaceDetector::Line2D ParkingSpaceDetector::make2D(const Line& L) { return make2D_from_Line(L); }

float ParkingSpaceDetector::perpendicularDistance(const Line2D& A, const Line2D& B) { return perpDist(A,B); }

float ParkingSpaceDetector::projectedOverlap(const Line2D& A, const Line2D& B) { return overlapAlongA(A,B); }

bool ParkingSpaceDetector::backAtLineEnds(const Line2D& A, const Line2D& B, const Line2D& C, float end_tol) {
  auto distPointToLine = [](const Eigen::Vector2f& q, const Line2D& L){
    Eigen::Vector2f n(-L.d.y(), L.d.x()); return std::abs((q - L.p).dot(n));
  };
  Eigen::Vector2f Aend = ((A.a - B.a).norm() < (A.b - B.b).norm()) ? A.a : A.b;
  Eigen::Vector2f Bend = ((B.a - A.a).norm() < (B.b - A.b).norm()) ? B.a : B.b;
  return distPointToLine(Aend, C) <= end_tol && distPointToLine(Bend, C) <= end_tol;
}

std::vector<ParkingSpaceDetector::ParkingSpace>
// Detect parking spaces from detected lines
ParkingSpaceDetector::findParkingSpaces(const std::vector<Line> &lines)
{
  std::vector<ParkingSpace> spaces;
  if (lines.size() < 3) return spaces;

  const float parallel_dot_min = 0.98f; // sides nearly parallel
  const float perp_dot_max     = 0.20f; // back nearly perpendicular
  const float width_tol        = static_cast<float>(width_tol_);
  const float min_overlap      = std::max(0.5f, static_cast<float>(overlap_ratio_ * parking_length_));
  const float end_tol          = static_cast<float>(end_tol_);

  std::vector<Line2D> L2; L2.reserve(lines.size());
  for (const auto& L : lines) L2.push_back(make2D(L));

  for (size_t i = 0; i < L2.size(); ++i) {
    for (size_t j = i + 1; j < L2.size(); ++j) {
      float dot = std::abs(L2[i].d.dot(L2[j].d));
      if (dot < parallel_dot_min) continue;

      float width = perpendicularDistance(L2[i], L2[j]);
      if (std::abs(width - static_cast<float>(parking_width_)) > width_tol) continue;

      float ov = std::min(projectedOverlap(L2[i], L2[j]),
                          projectedOverlap(L2[j], L2[i]));
      if (ov < min_overlap) continue;

      for (size_t k = 0; k < L2.size(); ++k) {
        if (k == i || k == j) continue;
        float dotp = std::abs(L2[k].d.dot(L2[i].d));
        if (dotp > perp_dot_max) continue;
        if (!backAtLineEnds(L2[i], L2[j], L2[k], end_tol)) continue;

        // Build rectangle
        ParkingSpace s;
        s.width  = width;
        s.length = ov;

        Eigen::Vector2f mid = 0.5f*(L2[i].p + L2[j].p);
        Eigen::Vector2f wv  = (L2[j].p - L2[i].p);
        if (wv.norm() > 1e-3f) wv.normalize(); else wv = Eigen::Vector2f(-L2[i].d.y(), L2[i].d.x());
        Eigen::Vector2f lv  = L2[i].d;

        float hw = 0.5f * s.width, hl = 0.5f * s.length;
        Eigen::Vector2f p1 = mid - wv*hw - lv*hl;
        Eigen::Vector2f p2 = mid + wv*hw - lv*hl;
        Eigen::Vector2f p3 = mid + wv*hw + lv*hl;
        Eigen::Vector2f p4 = mid - wv*hw + lv*hl;

        float gz = (L2[i].z + L2[j].z + L2[k].z) / 3.0f;
        s.center  = {mid.x(), mid.y(), gz};
        s.corner1 = {p1.x(), p1.y(), gz};
        s.corner2 = {p2.x(), p2.y(), gz};
        s.corner3 = {p3.x(), p3.y(), gz};
        s.corner4 = {p4.x(), p4.y(), gz};
        s.angle   = std::atan2(lv.y(), lv.x());

        spaces.push_back(s);
      }
    }
  }
  return spaces;
}
