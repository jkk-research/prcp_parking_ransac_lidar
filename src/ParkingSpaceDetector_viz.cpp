#include "ParkingSpaceDetector.hpp"

// clear markers used by the node (keeps visualization tidy)
void ParkingSpaceDetector::clearAllMarkers(const std_msgs::msg::Header &hdr)
{
  visualization_msgs::msg::MarkerArray arr;
  for (const char* ns : {"detected_lines", "parking_spaces", "parking_spaces_text"}) {
    visualization_msgs::msg::Marker m; m.header = hdr; m.ns = ns; m.action = visualization_msgs::msg::Marker::DELETEALL; arr.markers.push_back(m);
  }
  // Publish to bbox marker publisher as a safe place for clearing
  if (bbox_marker_pub_) bbox_marker_pub_->publish(arr);
}