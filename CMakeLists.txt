cmake_minimum_required(VERSION 3.5)
project(prcp_parking_ransac_lidar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Build 
add_executable(prcp_parking_ransac_lidar_node
  src/main.cpp
  src/ParkingSpaceDetector_ctor.cpp
  src/ParkingSpaceDetector_helpers.cpp
  src/ParkingSpaceDetector_callback.cpp
  src/ParkingSpaceDetector_detect.cpp
  src/ParkingSpaceDetector_merge.cpp
  src/ParkingSpaceDetector_parking.cpp
  src/ParkingSpaceDetector_viz.cpp
)

ament_target_dependencies(prcp_parking_ransac_lidar_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(prcp_parking_ransac_lidar_node
  ${PCL_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install node
install(TARGETS
  prcp_parking_ransac_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()
