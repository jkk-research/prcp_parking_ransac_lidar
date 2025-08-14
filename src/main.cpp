#include "ParkingSpaceDetector.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingSpaceDetector>());
  rclcpp::shutdown();
  return 0;
}
