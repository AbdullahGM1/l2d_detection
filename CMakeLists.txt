
cmake_minimum_required(VERSION 3.8)
project(ros2_depth_map_detection_localization_cpp)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(message_filters REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Add executable for the main depth map node
add_executable(depth_map_detection_localization src/depth_map_detection_localization.cpp)

# Link libraries
target_link_libraries(depth_map_detection_localization 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS} 
)

ament_target_dependencies(
  depth_map_detection_localization
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  PCL
  pcl_conversions
  yolo_msgs
  geometry_msgs
  message_filters
)

# Install the targets
install(TARGETS 
  depth_map_detection_localization
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_package()

