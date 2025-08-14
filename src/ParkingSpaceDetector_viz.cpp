#include "ParkingSpaceDetector.hpp"

// visualization messages
void ParkingSpaceDetector::publishDetectedLines(const std::vector<Line> &lines, const std_msgs::msg::Header &hdr)
{
  visualization_msgs::msg::MarkerArray arr;
  { visualization_msgs::msg::Marker m; m.header = hdr; m.ns = "detected_lines"; m.action = visualization_msgs::msg::Marker::DELETEALL; arr.markers.push_back(m); }
  int id = 0;
  for (const auto &ln : lines) {
    visualization_msgs::msg::Marker m;
    m.header = hdr; m.ns = "detected_lines"; m.id = id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.05;
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
    geometry_msgs::msg::Point p1, p2;
    p1.x = ln.start_point.x(); p1.y = ln.start_point.y(); p1.z = ln.start_point.z() + 0.2;
    p2.x = ln.end_point.x();   p2.y = ln.end_point.y();   p2.z = ln.end_point.z() + 0.2;
    m.points.push_back(p1); m.points.push_back(p2);
    arr.markers.push_back(m);
  }
  lines_pub_->publish(arr);
}

// publish parking space markers
void ParkingSpaceDetector::publishParkingMarkers(const std::vector<ParkingSpace> &spaces, const std_msgs::msg::Header &hdr)
{
  visualization_msgs::msg::MarkerArray arr;
  { visualization_msgs::msg::Marker m; m.header = hdr; m.ns = "parking_spaces"; m.action = visualization_msgs::msg::Marker::DELETEALL; arr.markers.push_back(m);
    visualization_msgs::msg::Marker mt; mt.header = hdr; mt.ns = "parking_spaces_text"; mt.action = visualization_msgs::msg::Marker::DELETEALL; arr.markers.push_back(mt); }
  int id = 0;
  for (const auto &s : spaces) {
    visualization_msgs::msg::Marker m;
    m.header = hdr; m.ns = "parking_spaces"; m.id = id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.15;
    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;

    geometry_msgs::msg::Point p1,p2,p3,p4;
    p1.x = s.corner1.x(); p1.y = s.corner1.y(); p1.z = s.corner1.z() + 0.2;
    p2.x = s.corner2.x(); p2.y = s.corner2.y(); p2.z = s.corner2.z() + 0.2;
    p3.x = s.corner3.x(); p3.y = s.corner3.y(); p3.z = s.corner3.z() + 0.2;
    p4.x = s.corner4.x(); p4.y = s.corner4.y(); p4.z = s.corner4.z() + 0.2;

    m.points.push_back(p3); m.points.push_back(p2); m.points.push_back(p1); m.points.push_back(p4); m.points.push_back(p3);
    arr.markers.push_back(m);

    visualization_msgs::msg::Marker t;
    t.header = hdr; t.ns = "parking_spaces_text"; t.id = id++;
    t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    t.action = visualization_msgs::msg::Marker::ADD;
    t.pose.position.x = s.center.x();
    t.pose.position.y = s.center.y();
    t.pose.position.z = s.center.z() + 1.0;
    t.scale.z = 0.5;
    t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0;
    t.text = "PARKING";
    arr.markers.push_back(t);
  }
  marker_pub_->publish(arr);
}

// clar out the markers as they tend to accumulate
void ParkingSpaceDetector::clearAllMarkers(const std_msgs::msg::Header &hdr)
{
  visualization_msgs::msg::MarkerArray arr;
  for (const std::string &ns : {"detected_lines", "parking_spaces", "parking_spaces_text"}) {
    visualization_msgs::msg::Marker m; m.header = hdr; m.ns = ns; m.action = visualization_msgs::msg::Marker::DELETEALL; arr.markers.push_back(m);
  }
  lines_pub_->publish(arr); marker_pub_->publish(arr);
}
