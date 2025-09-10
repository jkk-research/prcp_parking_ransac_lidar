#include "ParkingSpaceDetector.hpp"

#include <chrono>
using SteadyClock = std::chrono::steady_clock;

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
ParkingSpaceDetector::findParkingSpaces(const std::vector<Line> &lines)
{
  // start timer
  const auto t0 = SteadyClock::now();

  std::vector<ParkingSpace> spaces;
  if (lines.size() < 2) return spaces;

  const float parallel_dot_min = 0.96f;
  const float width_tol        = static_cast<float>(width_tol_);
  const float min_overlap      = std::max(0.3f, static_cast<float>(overlap_ratio_ * parking_length_));
  
  // New constraints to reduce false positives
  const float min_line_length_ratio = 0.6f; // Lines must be at least 60% of expected parking length
  const float max_line_length_ratio = 2.0f; // Lines shouldn't be too long either
  const float min_line_length = static_cast<float>(parking_length_) * min_line_length_ratio;
  const float max_line_length = static_cast<float>(parking_length_) * max_line_length_ratio;
  const float line_length_similarity = 0.7f; // Lines should be similar length
  const float width_tolerance_tight = std::min(width_tol, static_cast<float>(parking_width_) * 0.15f); // Max 15% width variation

  // Helper function to check if a line intersects or lies within a rectangle
  auto lineIntersectsRectangle = [](const Line2D& line, const Eigen::Vector2f& center, 
                                   float width, float length, float angle) -> bool {
    
    // Transform line endpoints to rectangle's local coordinate system
    float cos_a = std::cos(-angle); 
    float sin_a = std::sin(-angle);
    
    auto transformPoint = [&](const Eigen::Vector2f& p) -> Eigen::Vector2f {
      Eigen::Vector2f rel = p - center;
      return Eigen::Vector2f(
        rel.x() * cos_a - rel.y() * sin_a,
        rel.x() * sin_a + rel.y() * cos_a
      );
    };
    
    Eigen::Vector2f local_a = transformPoint(line.a);
    Eigen::Vector2f local_b = transformPoint(line.b);
    
    // Rectangle bounds in local coordinates (centered at origin)
    float half_width = width * 0.5f;
    float half_length = length * 0.5f;
    
    // Check if either endpoint is inside rectangle
    auto pointInBounds = [&](const Eigen::Vector2f& p) -> bool {
      return std::abs(p.x()) <= half_width && std::abs(p.y()) <= half_length;
    };
    
    if (pointInBounds(local_a) || pointInBounds(local_b)) {
      return true;
    }
    
    // Check if line intersects rectangle edges using bounds check
    // Line from local_a to local_b
    float min_x = std::min(local_a.x(), local_b.x());
    float max_x = std::max(local_a.x(), local_b.x());
    float min_y = std::min(local_a.y(), local_b.y());
    float max_y = std::max(local_a.y(), local_b.y());
    
    // Check if line's bounding box intersects rectangle
    if (max_x < -half_width || min_x > half_width || 
        max_y < -half_length || min_y > half_length) {
      return false;
    }
    
    auto lineSegmentIntersectsHorizontal = [&](float y_edge, float x_min, float x_max) -> bool {
      if (std::abs(local_a.y() - local_b.y()) < 1e-6f) return false; // Horizontal line, parallel to edge
      
      float t = (y_edge - local_a.y()) / (local_b.y() - local_a.y());
      if (t < 0.0f || t > 1.0f) return false;
      
      float x_intersect = local_a.x() + t * (local_b.x() - local_a.x());
      return x_intersect >= x_min && x_intersect <= x_max;
    };
    
    auto lineSegmentIntersectsVertical = [&](float x_edge, float y_min, float y_max) -> bool {
      if (std::abs(local_a.x() - local_b.x()) < 1e-6f) return false; // Vertical line, parallel to edge
      
      float t = (x_edge - local_a.x()) / (local_b.x() - local_a.x());
      if (t < 0.0f || t > 1.0f) return false;
      
      float y_intersect = local_a.y() + t * (local_b.y() - local_a.y());
      return y_intersect >= y_min && y_intersect <= y_max;
    };
    
    // Check intersection with all 4 edges
    return lineSegmentIntersectsHorizontal(-half_length, -half_width, half_width) ||  // Bottom edge
           lineSegmentIntersectsHorizontal(half_length, -half_width, half_width) ||   // Top edge
           lineSegmentIntersectsVertical(-half_width, -half_length, half_length) ||   // Left edge
           lineSegmentIntersectsVertical(half_width, -half_length, half_length);      // Right edge
  };

  std::vector<Line2D> L2; L2.reserve(lines.size());
  for (const auto& L : lines) L2.push_back(make2D(L));

  for (size_t i = 0; i < L2.size(); ++i) {
    for (size_t j = i + 1; j < L2.size(); ++j) {
      
      // 1. Check parallelism 
      float dot = std::abs(L2[i].d.dot(L2[j].d));
      if (dot < parallel_dot_min) continue;

      // 2. Check individual line lengths
      float len_i = (L2[i].b - L2[i].a).norm();
      float len_j = (L2[j].b - L2[j].a).norm();
      
      if (len_i < min_line_length || len_i > max_line_length) continue;
      if (len_j < min_line_length || len_j > max_line_length) continue;
      
      // 3. Check line length similarity
      float length_ratio = std::min(len_i, len_j) / std::max(len_i, len_j);
      if (length_ratio < line_length_similarity) continue;

      // 4. Check width
      float width = perpendicularDistance(L2[i], L2[j]);
      if (std::abs(width - static_cast<float>(parking_width_)) > width_tolerance_tight) continue;

      // 5. Check overlap
      float ov_i_on_j = projectedOverlap(L2[i], L2[j]);
      float ov_j_on_i = projectedOverlap(L2[j], L2[i]);
      float ov = std::min(ov_i_on_j, ov_j_on_i);
      if (ov < min_overlap) continue;
      
      // 6. Overlap should be substantial relative to line lengths
      float overlap_ratio_i = ov / len_i;
      float overlap_ratio_j = ov / len_j;
      if (overlap_ratio_i < 0.6f || overlap_ratio_j < 0.6f) continue; // At least 60% overlap
      
      // 7. Lines should be roughly aligned (not offset too much)
      Eigen::Vector2f mid_i = 0.5f * (L2[i].a + L2[i].b);
      Eigen::Vector2f mid_j = 0.5f * (L2[j].a + L2[j].b);
      Eigen::Vector2f offset_vec = mid_j - mid_i;
      
      // Project offset onto line direction - should be small for well-aligned lines
      float along_line_offset = std::abs(offset_vec.dot(L2[i].d));
      float max_along_offset = std::min(len_i, len_j) * 0.3f; // Max 30% of line length
      if (along_line_offset > max_along_offset) continue;
      
      // 8. New check: Aspect ratio should be reasonable for parking space
      float aspect_ratio = ov / width;
      float expected_aspect = static_cast<float>(parking_length_) / static_cast<float>(parking_width_);
      float aspect_tolerance = expected_aspect * 0.4f; // Allow 40% variation
      if (std::abs(aspect_ratio - expected_aspect) > aspect_tolerance) continue;
      
      // 9. Line endpoints should create reasonable rectangle
      // Check if lines are positioned to form proper sides of rectangle
      Eigen::Vector2f perp_dir = Eigen::Vector2f(-L2[i].d.y(), L2[i].d.x());
      if ((L2[j].p - L2[i].p).dot(perp_dir) < 0) perp_dir = -perp_dir;
      
      // Build rectangle from the two sides
      ParkingSpace s;
      s.width  = width;
      s.length = ov;

      Eigen::Vector2f mid = 0.5f*(L2[i].p + L2[j].p);
      Eigen::Vector2f wv  = (L2[j].p - L2[i].p);
      if (wv.norm() > 1e-3f) wv.normalize(); 
      else wv = Eigen::Vector2f(-L2[i].d.y(), L2[i].d.x());
      Eigen::Vector2f lv  = L2[i].d;

      float hw = 0.5f * s.width, hl = 0.5f * s.length;
      Eigen::Vector2f p1 = mid - wv*hw - lv*hl;
      Eigen::Vector2f p2 = mid + wv*hw - lv*hl;
      Eigen::Vector2f p3 = mid + wv*hw + lv*hl;
      Eigen::Vector2f p4 = mid - wv*hw + lv*hl;

      // With no back line, use the average height of the two side lines
      float gz = (L2[i].z + L2[j].z) / 2.0f;
      s.center  = {mid.x(), mid.y(), gz};
      s.corner1 = {p1.x(), p1.y(), gz};
      s.corner2 = {p2.x(), p2.y(), gz};
      s.corner3 = {p3.x(), p3.y(), gz};
      s.corner4 = {p4.x(), p4.y(), gz};
      s.angle   = std::atan2(lv.y(), lv.x());

      // Check for additional lines within the parking space (noise detection)
      // Count OTHER lines (not the two we used to build this space) that intersect
      int other_lines_in_space = 0;
      Eigen::Vector2f space_center(mid.x(), mid.y());
      
      for (size_t k = 0; k < L2.size(); ++k) {
        // Skip the two lines we used to construct this parking space
        if (k == i || k == j) continue;
        
        if (lineIntersectsRectangle(L2[k], space_center, s.width, s.length, s.angle)) {
          other_lines_in_space++;
        }
      }
      
      // If 3 or more OTHER lines intersect this space, it's likely noise
      // Real parking spaces should be clean areas with just the boundary lines
      if (other_lines_in_space < 3) {
        spaces.push_back(s);
      }
    }
  }
  
  // Post-processing: Remove spaces that are too close to each other
  // (Keep the one with better geometry score)
  const float min_separation = static_cast<float>(parking_width_) * 0.8f; // Spaces must be at least 80% of width apart
  
  std::vector<bool> keep(spaces.size(), true);
  
  for (size_t i = 0; i < spaces.size(); ++i) {
    if (!keep[i]) continue;
    
    for (size_t j = i + 1; j < spaces.size(); ++j) {
      if (!keep[j]) continue;
      
      // Calculate distance between space centers
      float dx = spaces[i].center.x() - spaces[j].center.x();
      float dy = spaces[i].center.y() - spaces[j].center.y();
      float distance = std::sqrt(dx*dx + dy*dy);
      
      if (distance < min_separation) {
        // Keep the space with better aspect ratio
        float expected_aspect = static_cast<float>(parking_length_) / static_cast<float>(parking_width_);
        float aspect_i = spaces[i].length / spaces[i].width;
        float aspect_j = spaces[j].length / spaces[j].width;
        
        float error_i = std::abs(aspect_i - expected_aspect);
        float error_j = std::abs(aspect_j - expected_aspect);
        
        if (error_i < error_j) {
          keep[j] = false;
        } else {
          keep[i] = false;
          break; // Move to next i since this one is marked for removal
        }
      }
    }
  }
  
  // Remove filtered spaces
  std::vector<ParkingSpace> filtered;
  for (size_t i = 0; i < spaces.size(); ++i) {
    if (keep[i]) filtered.push_back(spaces[i]);
  }
  

  const auto t1 = SteadyClock::now();

  // const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  // RCLCPP_INFO(this->get_logger(), "Parking space detection took %.3f ms", ms);
  return filtered;
}