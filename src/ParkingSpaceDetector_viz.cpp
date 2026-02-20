#include "ParkingSpaceDetector.hpp"

// clear markers used by the node (keeps visualization tidy)
void ParkingSpaceDetector::clearAllMarkers(const std_msgs::msg::Header &hdr)
{
  // Clear detected_lines on its own publisher
  if (detected_lines_pub_) {
    visualization_msgs::msg::MarkerArray lines_arr;
    visualization_msgs::msg::Marker m; m.header = hdr; m.ns = "detected_lines"; m.action = visualization_msgs::msg::Marker::DELETEALL;
    lines_arr.markers.push_back(m);
    detected_lines_pub_->publish(lines_arr);
  }

  // Clear parking spaces on its own publisher
  if (parking_spaces_pub_) {
    visualization_msgs::msg::MarkerArray ps_arr;
    visualization_msgs::msg::Marker m; m.header = hdr; m.ns = "parking_spaces"; m.action = visualization_msgs::msg::Marker::DELETEALL;
    ps_arr.markers.push_back(m);
    parking_spaces_pub_->publish(ps_arr);
  }
}