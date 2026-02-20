#include "ParkingSpaceDetector.hpp"
#include <cmath>

void ParkingSpaceDetector::findParkingSpaces(const std::vector<LineSegment> &lines,
                                             const std_msgs::msg::Header &header)
{
  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker del;
  del.header = header;
  del.ns     = "parking_spaces";
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  if (!parking_detection_enabled_ || lines.size() < 2) {
    parking_spaces_pub_->publish(arr);
    return;
  }

  const float cos_tol = std::cos(static_cast<float>(parking_angle_tolerance_)
                                 * static_cast<float>(M_PI) / 180.0f);
  const float w_min  = static_cast<float>(parking_width_min_);
  const float w_max  = static_cast<float>(parking_width_max_);
  const float l_min  = static_cast<float>(parking_length_min_);
  const float l_max  = static_cast<float>(parking_length_max_);
  const float ll_min = static_cast<float>(parking_line_length_min_);

  std::vector<bool> used(lines.size(), false);
  int ps_id = 0;

  for (size_t i = 0; i < lines.size(); ++i) {
    if (used[i]) continue;
    const LineSegment &li = lines[i];
    if (li.length() < ll_min) continue;

    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (used[j]) continue;
      const LineSegment &lj = lines[j];
      if (lj.length() < ll_min) continue;

      // Parallel check
      if (std::abs(li.dir.dot(lj.dir)) < cos_tol) continue;

      // Align direction signs so both point "the same way".
      Eigen::Vector2f dj = (li.dir.dot(lj.dir) >= 0.0f) ? lj.dir : -lj.dir;
      (void)dj; // direction used only for sign-alignment, not needed further

      // Perpendicular (width) check
      Eigen::Vector2f perp(-li.dir.y(), li.dir.x());
      float signed_w = (lj.centroid - li.centroid).dot(perp);
      float width    = std::abs(signed_w);
      if (width < w_min || width > w_max) continue;

      // Longitudinal overlap check (?)
      float s    = (lj.centroid - li.centroid).dot(li.dir);
      float j_lo = s + lj.min_proj;
      float j_hi = s + lj.max_proj;
      float o_lo = std::max(li.min_proj, j_lo);
      float o_hi = std::min(li.max_proj, j_hi);
      float overlap = o_hi - o_lo;
      if (overlap < l_min || overlap > l_max) continue;

      // Valid parking space found.
      used[i] = used[j] = true;
      float mean_z = (li.mean_z + lj.mean_z) * 0.5f + 0.05f; // slight Z lift

      Eigen::Vector2f c1 = li.centroid + o_lo * li.dir;
      Eigen::Vector2f c2 = li.centroid + o_hi * li.dir;
      Eigen::Vector2f c3 = c2 + signed_w * perp;
      Eigen::Vector2f c4 = c1 + signed_w * perp;
      Eigen::Vector2f center = (c1 + c2 + c3 + c4) * 0.25f;

      // Helper to convert a 2-D corner to a ROS point at mean_z.
      auto to_pt = [&](const Eigen::Vector2f &v) {
        geometry_msgs::msg::Point p;
        p.x = v.x(); p.y = v.y(); p.z = mean_z;
        return p;
      };

      visualization_msgs::msg::Marker rect;
      rect.header = header;
      rect.ns     = "parking_spaces";
      rect.id     = ps_id * 2;
      rect.type   = visualization_msgs::msg::Marker::LINE_STRIP;
      rect.action = visualization_msgs::msg::Marker::ADD;
      rect.scale.x = 0.08;
      rect.color.r = 1.0f; rect.color.g = 1.0f; rect.color.b = 0.0f; rect.color.a = 1.0f;
      rect.pose.orientation.w = 1.0;
      rect.points.push_back(to_pt(c1));
      rect.points.push_back(to_pt(c2));
      rect.points.push_back(to_pt(c3));
      rect.points.push_back(to_pt(c4));
      rect.points.push_back(to_pt(c1)); // close the loop
      arr.markers.push_back(rect);

      visualization_msgs::msg::Marker txt;
      txt.header = header;
      txt.ns     = "parking_spaces";
      txt.id     = ps_id * 2 + 1;
      txt.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x = center.x();
      txt.pose.position.y = center.y();
      txt.pose.position.z = mean_z + 0.5f;
      txt.pose.orientation.w = 1.0;
      txt.scale.z = 0.4;
      txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
      txt.text = "PARKING";
      arr.markers.push_back(txt);

      ++ps_id;
      break;
    }
  }

  parking_spaces_pub_->publish(arr);
}
